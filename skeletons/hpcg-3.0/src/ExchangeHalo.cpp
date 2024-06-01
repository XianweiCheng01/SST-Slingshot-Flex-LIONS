
//@HEADER
// ***************************************************
//
// HPCG: High Performance Conjugate Gradient Benchmark
//
// Contact:
// Michael A. Heroux ( maherou@sandia.gov)
// Jack Dongarra     (dongarra@eecs.utk.edu)
// Piotr Luszczek    (luszczek@eecs.utk.edu)
//
// ***************************************************
//@HEADER

/*!
 @file ExchangeHalo.cpp

 HPCG routine
 */

// Compile this routine only if running with MPI
#ifndef HPCG_NO_MPI
#include <mpi.h>
#include "Geometry.hpp"
#include "ExchangeHalo.hpp"
#include <cstdlib>

#include <iostream>
#include "mytimer.hpp"

/*!
  Communicates data that is at the border of the part of the domain assigned to this processor.

  @param[in]    A The known system matrix
  @param[inout] x On entry: the local vector entries followed by entries to be communicated; on exit: the vector with non-local entries updated by other processors
 */
void ExchangeHalo(const SparseMatrix & A, Vector & x) {

  // Extract Matrix pieces

  std::cout << "Test time ExchangeHalo 00 in rank: " << A.geom->rank << " at " << mytimer() << std::endl;
  local_int_t localNumberOfRows = A.localNumberOfRows;
  int num_neighbors = A.numberOfSendNeighbors;
  local_int_t * receiveLength = A.receiveLength;
  local_int_t * sendLength = A.sendLength;
  int * neighbors = A.neighbors;
  double * sendBuffer = A.sendBuffer;
  local_int_t totalToBeSent = A.totalToBeSent;
  local_int_t * elementsToSend = A.elementsToSend;

  double * const xv = x.values;

  //
  //  first post receives, these are immediate receives
  //  Do not wait for result to come, will do that at the
  //  wait call below.
  //

  int MPI_MY_TAG = 99;

  MPI_Request * request = new MPI_Request[num_neighbors];

  //
  // Externals are at end of locals
  //
  double * x_external = (double *) xv + localNumberOfRows;
  std::cout << "Test time ExchangeHalo 01 in rank: " << A.geom->rank << " at " << mytimer() << std::endl;

  // Post receives first
  // TODO: Thread this loop
  for (int i = 0; i < num_neighbors; i++) {
    local_int_t n_recv = receiveLength[i];
    //printf("Test in ExchangeHalo with: %d, %d, %d\n", n_recv, neighbors[i], request);
    MPI_Irecv(x_external, n_recv, MPI_DOUBLE, neighbors[i], MPI_MY_TAG, MPI_COMM_WORLD, request+i);
    x_external += n_recv;
  }

  std::cout << "Test time ExchangeHalo 02 in rank: " << A.geom->rank << " at " << mytimer() << std::endl;

  //
  // Fill up send buffer
  //

  // TODO: Thread this loop
#pragma sst compute
  for (local_int_t i=0; i<totalToBeSent; i++) sendBuffer[i] = xv[elementsToSend[i]];

  //
  // Send to each neighbor
  //
  std::cout << "Test time ExchangeHalo 03 in rank: " << A.geom->rank << " at " << mytimer() << std::endl;

  // TODO: Thread this loop
  for (int i = 0; i < num_neighbors; i++) {
    local_int_t n_send = sendLength[i];
    MPI_Send(sendBuffer, n_send, MPI_DOUBLE, neighbors[i], MPI_MY_TAG, MPI_COMM_WORLD);
#pragma sst delete
    sendBuffer += n_send;
  }
  std::cout << "Test time ExchangeHalo 04 in rank: " << A.geom->rank << " at " << mytimer() << std::endl;

  //
  // Complete the reads issued above
  //

  MPI_Status status;
  // TODO: Thread this loop
  for (int i = 0; i < num_neighbors; i++) {
    if ( MPI_Wait(request+i, &status) ) {
      std::exit(-1); // TODO: have better error exit
    }
  }
  std::cout << "Test time ExchangeHalo 05 in rank: " << A.geom->rank << " at " << mytimer() << std::endl;

  delete [] request;

  return;
}
#endif
// ifndef HPCG_NO_MPI
