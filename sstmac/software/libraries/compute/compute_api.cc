/**
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
*/

#include <sstmac/software/process/operating_system.h>
#include <sstmac/software/process/thread.h>
#include <sstmac/software/process/app.h>
#include <sstmac/software/process/time.h>
#include <sstmac/software/libraries/compute/compute_api.h>
#include <sprockit/thread_safe_new.h>
#include <sstmac/skeleton.h>


using sstmac::TimeDelta;
using sstmac::Timestamp;
using os = sstmac::sw::OperatingSystem;

extern "C" double sstmac_block()
{
  os::currentOs()->block();
  return os::currentOs()->now().sec();
}

extern "C" unsigned int sstmac_sleep(unsigned int secs){
  os::currentOs()->sleep(TimeDelta(secs, TimeDelta::one_second));
  return 0;
}

extern "C" unsigned sstmac_sleepUntil(double t){
  os::currentOs()->sleepUntil(Timestamp(t));
  return 0;
}

extern "C" int sstmac_usleep(unsigned int usecs){
  os::currentOs()->sleep(TimeDelta(usecs, TimeDelta::one_microsecond));
  return 0;
}

extern "C" int sstmac_nanosleep(unsigned int nanosecs){
  os::currentOs()->sleep(TimeDelta(nanosecs, TimeDelta::one_nanosecond));
  return 0;
}

extern "C" int sstmac_msleep(unsigned int msecs){
  os::currentOs()->sleep(TimeDelta(msecs, TimeDelta::one_millisecond));
  return 0;
}

extern "C" int sstmac_fsleep(double secs){
  sstmac::sw::OperatingSystem::currentThread()->parentApp()->sleep(sstmac::TimeDelta(secs));
  return 0;
}

extern "C" void sstmac_compute(double secs){
  sstmac::sw::OperatingSystem::currentOs()->compute(TimeDelta(secs));
}

//extern "C" void sstmac_memread(uint64_t bytes, const char* func)
extern "C" void sstmac_memread(uint64_t bytes)
{
  //printf("Test 0 in sstmac_memread of compute/compute_api.cc calling from\n");
  sstmac::sw::OperatingSystem::currentThread()->parentApp()
    ->computeBlockRead(bytes);
}

extern "C" void sstmac_memwrite(uint64_t bytes){
  sstmac::sw::OperatingSystem::currentThread()->parentApp()
    ->computeBlockWrite(bytes);
}

//extern "C" void sstmac_memcopy(uint64_t bytes, const char* func)
extern "C" void sstmac_memcopy(uint64_t bytes)
{
  //printf("Test 0 in sstmac_memcopy of compute/compute_api.cc calling from\n");
  std::cout << "Test 0 in sstmac_memcopy of compute/compute_api.cc with " << bytes << std::endl;
  sstmac::sw::OperatingSystem::currentThread()->parentApp()
    ->computeBlockMemcpy(bytes);
}

extern "C" void sstmac_compute_detailed(uint64_t nflops, uint64_t nintops, uint64_t bytes){
  //std::cout << "Test in sstmac_compute_detailed of compute_api.cc with bytes: " << bytes << " flops: " << nflops << " intops: " << nintops << std::endl;
  sstmac::sw::OperatingSystem::currentThread()
    ->computeDetailed(nflops, nintops, bytes);
}

extern "C" void sstmac_compute_detailed_nthr(uint64_t nflops, uint64_t nintops, uint64_t bytes,
                                        int nthread){
  //std::cout << "Test in sstmac_compute_detailed_nthr of compute_api.cc with bytes: " << bytes << " flops: " << nflops << " intops: " << nintops << " nth: " << nthread << std::endl;
  sstmac::sw::OperatingSystem::currentThread()
    ->computeDetailed(nflops, nintops, bytes, nthread);
}

extern "C" void sstmac_computeLoop(uint64_t num_loops, uint32_t nflops_per_loop,
                    uint32_t nintops_per_loop, uint32_t bytes_per_loop){
  //std::cout << "Test in sstmac_computeLoop of compute_api.cc with bytes: " << bytes_per_loop << " flops: " << nflops_per_loop << " intops: " << nintops_per_loop << " n_loop: " << num_loops << std::endl;
  sstmac::sw::OperatingSystem::currentThread()->parentApp()
    ->computeLoop(num_loops, nflops_per_loop, nintops_per_loop, bytes_per_loop);
}

extern "C" void sstmac_compute_loop2(uint64_t isize, uint64_t jsize,
                    uint32_t nflops_per_loop,
                    uint32_t nintops_per_loop, uint32_t bytes_per_loop){
  uint64_t num_loops = isize * jsize;
  //std::cout << "Test in sstmac_compute_loop2 of compute_api.cc with bytes: " << bytes_per_loop << " flops: " << nflops_per_loop << " intops: " << nintops_per_loop << " n_loop: " << num_loops << std::endl;
  sstmac::sw::OperatingSystem::currentThread()->parentApp()
    ->computeLoop(num_loops, nflops_per_loop, nintops_per_loop, bytes_per_loop);
}

extern "C" void
sstmac_compute_loop3(uint64_t isize, uint64_t jsize, uint64_t ksize,
                    uint32_t nflops_per_loop,
                    uint32_t nintops_per_loop,
                    uint32_t bytes_per_loop){
  uint64_t num_loops = isize * jsize * ksize;
  //std::cout << "Test in sstmac_compute_loop3 of compute_api.cc with bytes: " << bytes_per_loop << " flops: " << nflops_per_loop << " intops: " << nintops_per_loop << " n_loop: " << num_loops << std::endl;
  sstmac::sw::OperatingSystem::currentThread()->parentApp()
    ->computeLoop(num_loops, nflops_per_loop, nintops_per_loop, bytes_per_loop);
}

extern "C" void
sstmac_compute_loop4(uint64_t isize, uint64_t jsize, uint64_t ksize, uint64_t lsize,
                     uint32_t nflops_per_loop,
                     uint32_t nintops_per_loop,
                     uint32_t bytes_per_loop){
  uint64_t num_loops = isize * jsize * ksize * lsize;
  //std::cout << "Test in sstmac_compute_loop4 of compute_api.cc with bytes: " << bytes_per_loop << " flops: " << nflops_per_loop << " intops: " << nintops_per_loop << " n_loop: " << num_loops << std::endl;
  sstmac::sw::OperatingSystem::currentThread()->parentApp()
    ->computeLoop(num_loops, nflops_per_loop, nintops_per_loop, bytes_per_loop);
}
