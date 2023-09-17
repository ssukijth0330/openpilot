#!/usr/bin/env python3
import av
import cv2
import os
import sys
import argparse
import numpy as np
import time
import json

import cereal.messaging as messaging
from openpilot.tools.bodyteleop.yolo_helpers import YoloRunner

W, H = 1928, 1208
V4L2_BUF_FLAG_KEYFRAME = 8

# TODO: this is similar to openpilot/tools/camerastream/compressed_vipc.py
# refactor to share code!
def yolo_processor(addr, sock_name, yolo_runner, nvidia, debug=False):

  if debug:
    print("start decoder for %s" % sock_name)

  if nvidia:
    os.environ["NV_LOW_LATENCY"] = "3"    # both bLowLatency and CUVID_PKT_ENDOFPICTURE
    sys.path += os.environ["LD_LIBRARY_PATH"].split(":")
    import PyNvCodec as nvc

    nvDec = nvc.PyNvDecoder(W, H, nvc.PixelFormat.NV12, nvc.CudaVideoCodec.HEVC, 0)
    cc1 = nvc.ColorspaceConversionContext(nvc.ColorSpace.BT_709, nvc.ColorRange.JPEG)
    conv_yuv = nvc.PySurfaceConverter(W, H, nvc.PixelFormat.NV12, nvc.PixelFormat.YUV420, 0)
    nvDwn_yuv = nvc.PySurfaceDownloader(W, H, nvc.PixelFormat.YUV420, 0)
    img_yuv = np.ndarray((H*W//2*3), dtype=np.uint8)
  else:
    codec = av.CodecContext.create("hevc", "r")

  os.environ["ZMQ"] = "1"
  messaging.context = messaging.Context()
  sock = messaging.sub_sock(sock_name, None, addr=addr, conflate=False)
  psock = messaging.pub_sock('bodyReserved0')
  last_idx = -1
  seen_iframe = False

  time_q = []
  while 1:
    msgs = messaging.drain_sock(sock, wait_for_one=True)
    for evt in msgs:
      evta = getattr(evt, evt.which())
      if debug and evta.idx.encodeId != 0 and evta.idx.encodeId != (last_idx+1):
        print("DROP PACKET!")
      last_idx = evta.idx.encodeId
      if not seen_iframe and not (evta.idx.flags & V4L2_BUF_FLAG_KEYFRAME):
        if debug:
          print("waiting for iframe")
        continue
      time_q.append(time.monotonic())
      network_latency = (int(time.time()*1e9) - evta.unixTimestampNanos)/1e6
      frame_latency = ((evta.idx.timestampEof/1e9) - (evta.idx.timestampSof/1e9))*1000
      process_latency = ((evt.logMonoTime/1e9) - (evta.idx.timestampEof/1e9))*1000

      # put in header (first)
      if not seen_iframe:
        if nvidia:
          nvDec.DecodeSurfaceFromPacket(np.frombuffer(evta.header, dtype=np.uint8))
        else:
          codec.decode(av.packet.Packet(evta.header))
        seen_iframe = True

      if nvidia:
        rawSurface = nvDec.DecodeSurfaceFromPacket(np.frombuffer(evta.data, dtype=np.uint8))
        if rawSurface.Empty():
          if debug:
            print("DROP SURFACE")
          continue
        convSurface = conv_yuv.Execute(rawSurface, cc1)
        nvDwn_yuv.DownloadSingleSurface(convSurface, img_yuv)
      else:
        frames = codec.decode(av.packet.Packet(evta.data))
        if len(frames) == 0:
          if debug:
            print("DROP SURFACE")
          continue
        assert len(frames) == 1
        img_yuv = frames[0].to_ndarray(format=av.video.format.VideoFormat('yuv420p')).flatten()
        uv_offset = H*W
        y = img_yuv[:uv_offset]
        uv = img_yuv[uv_offset:].reshape(2, -1).ravel('F')
        img_yuv = np.hstack((y, uv))

      img_rgb = cv2.cvtColor(img_yuv.reshape(H*3//2, W), cv2.COLOR_YUV2RGB_NV12)
      outs = yolo_runner.run(img_rgb)

      dat = messaging.new_message()
      dat.bodyReserved0 = json.dumps(outs)
      psock.send(dat.to_bytes())

      pc_latency = (time.monotonic()-time_q[0])*1000
      time_q = time_q[1:]
      if debug:
        print("%2d %4d %.3f %.3f roll %6.2f ms latency %6.2f ms + %6.2f ms + %6.2f ms = %6.2f ms"
              % (len(msgs), evta.idx.encodeId, evt.logMonoTime/1e9, evta.idx.timestampEof/1e6, frame_latency,
                 process_latency, network_latency, pc_latency, process_latency+network_latency+pc_latency ), len(evta.data), sock_name)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='')
  parser.add_argument("addr", help="Address of comma three")
  args = parser.parse_args()
  socket_name = 'driverEncodeData'
  # TODO: download from from https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx
  # place it in openpilot/tools/bodyteleop/models
  yolo_runner = YoloRunner('models/yolov5n.onnx')
  yolo_processor(args.addr, socket_name, yolo_runner, nvidia=False, debug=False)