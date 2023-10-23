#!/usr/bin/env python3
import os
import enum
import fnmatch
import subprocess
import pandas

from collections import Counter, defaultdict
from typing import NamedTuple

from common.basedir import BASEDIR

LANG = {
  "Python": ['*.py'],
  "Cython": ['*.pxd', '*.pyx', '*.'],
  "C": ['*.cpp', '*.c', '*.cc'],
  "C Header": ['*.h', '*.hpp'],
  "SCons": ['SConstruct', 'SConscript'],
  "Docs": ['*.md', 'README'],
  "YAML": ['*.yaml'],
  "Shell": ['*.sh'],
  "Docker": ['Dockerfile*'],
  "Translations": ['*.ts'],
  "SVG": ['*.svg'],
  "JSON": ['*.json'],
  "OpenCL": ['*.cl'],
  "FW versions": ['values.py'],
}

FileType = enum.Enum('FileType', ['real', 'test', 'dependency', 'support', 'tools'])

class File(NamedTuple):
  fn: str
  language: str
  typ: FileType
  lines: int

  @staticmethod
  def from_fn(fn):
    lang = "unknown"
    for k, pats in LANG.items():
      for p in pats:
        if fnmatch.fnmatch(fn, p) or fnmatch.fnmatch(fn, '*/'+p):
          lang = k

    with open(fn, 'rb') as f:
      dat = f.read()
      if b'\0' in dat:
        count = 0
        lang = "Binary"
      else:
        count = len(dat.strip().splitlines())

    return File(fn, lang, get_type(fn), count)

def get_type(fn):
  if fn.startswith('third_party/'):
    return FileType.dependency
  if fn.startswith(('tools/', 'scripts/')):
    return FileType.tools
  if "tests/" in fn or "test/" in fn or "test_" in fn or fn in ('Jenkinsfile', ):
    return FileType.test
  if fn.startswith('.github/') or fn.startswith('docs') or fn in ('poetry.lock', 'pyproject.toml'):
    return FileType.support
  return FileType.real


if __name__ == "__main__":
  o = subprocess.check_output("git ls-files", cwd=BASEDIR, shell=True, encoding='utf8')
  raw_files = o.strip().splitlines()

  # TODO: handle these, and also recurse submodules
  raw_files = [f for f in raw_files if os.path.isfile(f)]

  files = [File.from_fn(f) for f in raw_files]
  stats = defaultdict(lambda: defaultdict(int))
  for f in files:
    stats[f.typ][f.language] += f.lines
    stats[f.typ]['files'] += 1
    stats[f.typ]['overall lines'] += f.lines

  for t in FileType:
    print("="*10, str(t).replace('FileType.', '').center(10), "="*10)
    for k, v in stats[t].items():
      print(k.ljust(25), v)
    print()

  # TODO: lines by daemon
  # TODO: line percent by file type
  print("overall")
  print(len(files), "total files")

  if "DEBUG" in os.environ:
    for f in files:
      if f.typ == FileType.real and f.language == "unknown" and f.lines > 100:
        print(f)
