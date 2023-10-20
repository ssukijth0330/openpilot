import onnx
import codecs
import pickle
import pathlib
from typing import Tuple, Dict

def get_name_and_shape(value_info:onnx.ValueInfoProto) -> Tuple[str, Tuple[int,...]]:
  shape = tuple([int(dim.dim_value) for dim in value_info.type.tensor_type.shape.dim])
  name = value_info.name
  return name, shape

def get_model_metadata(model_path:pathlib.Path) -> Dict:
  model = onnx.load(str(model_path))
  i = [x.key for x in model.metadata_props].index('output_slices')
  output_slices = model.metadata_props[i].value

  metadata = {}
  metadata['output_slices'] = pickle.loads(codecs.decode(output_slices.encode(), "base64"))
  metadata['input_shapes'] = dict([get_name_and_shape(x) for x in model.graph.input])
  metadata['output_shapes'] = dict([get_name_and_shape(x) for x in model.graph.output])
  return metadata
