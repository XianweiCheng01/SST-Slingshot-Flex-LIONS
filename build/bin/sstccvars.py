"""
Copyright 2009-2022 National Technology and Engineering Solutions of Sandia,
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S. Government
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly
owned subsidiary of Honeywell International, Inc., for the U.S. Department of
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2022, NTESS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Questions? Contact sst-macro-help@sandia.gov
"""
sstLdFlags = [
' -Wl,--no-as-needed',
'-L${exec_prefix}/lib',
]

haveFloat128=False
clangCppFlagsStr="-I/home/xianwei/local/clang-llvm//include"
clangLdFlagsStr="-L/home/xianwei/local/clang-llvm//lib"
clangLibtoolingCxxFlagsStr="-I/home/xianwei/local/clang-llvm/lib/clang/17/include -I/include "
clangLibtoolingCFlagsStr="-I/home/xianwei/local/clang-llvm/lib/clang/17/include -I/include "
defaultIncludePaths="/usr/lib/gcc/x86_64-linux-gnu/9/../../../../include/c++/9:/usr/lib/gcc/x86_64-linux-gnu/9/../../../../include/x86_64-linux-gnu/c++/9:/usr/lib/gcc/x86_64-linux-gnu/9/../../../../include/c++/9/backward:/home/xianwei/local/clang-llvm/lib/clang/17/include:/usr/local/include:/usr/include/x86_64-linux-gnu:/include:/usr/include"
clangBin="/home/xianwei/local/clang-llvm/"

sstCppFlags = [
"-DSSTMAC_INTEGRATED_SST_CORE=1 -I/home/xianwei/local/sstcore/include -I/home/xianwei/local/sstcore/include/sst -I/home/xianwei/local/sstcore/include/sst/core -D__STDC_FORMAT_MACROS",
"",
"-I${includedir}/sstmac",
"-I${includedir}",
"-I${includedir}/include",
"-I${includedir}/memoization",
"-I${includedir}/sstmac/software/libraries",
"-I${includedir}/sstmac/tools",
"-I${includedir}/sumi",
"-DSSTMAC=1",
]

clangDir="/home/xianwei/local/clang-llvm/"

sstCore=bool("-DSSTMAC_INTEGRATED_SST_CORE=1 -I/home/xianwei/local/sstcore/include -I/home/xianwei/local/sstcore/include/sst -I/home/xianwei/local/sstcore/include/sst/core -D__STDC_FORMAT_MACROS")
soFlagsStr="-shared -undefined dynamic_lookup"

srcDir="/home/xianwei/new_source/sst-macro/build/.."
buldDir="/home/xianwei/new_source/sst-macro/build"
prefix="/home/xianwei/local/sstmacro"
execPrefix="${prefix}"
includeDir="${prefix}/include"
sstStdFlag="-std=c++17"
sstCxxFlagsStr="-g -O2   -D__STDC_FORMAT_MACROS -std=c++17"
sstCFlagsStr="-g -O2"
cc="clang"
cxx="clang++"
spackcc=""
spackcxx=""

# Configure the PYTHONPATH
def setPythonPath():
  relpath = inspect.getfile(inspect.currentframe()) # script filename (usually with path)
  abspath = os.path.abspath(relpath)
  my_folder = os.path.dirname(abspath)

  my_sstmac_include_dir = None

  my_src_folder = os.path.join(my_folder, "pysst")
  if (os.path.isdir(my_src_folder)):
      my_sstmac_include_dir = os.path.join(*os.path.split(my_folder)[:-1])
      sys.path.append(my_folder)

  my_inc_folder = os.path.join(*os.path.split(my_folder)[:-1])
  my_inc_folder = os.path.join(my_inc_folder, "include", "sstmac")
  if (os.path.isdir(my_inc_folder)):
      sys.path.append(my_inc_folder)
      if not my_sstmac_include_dir:
          my_sstmac_include_dir = os.path.join(my_inc_folder, "include")




