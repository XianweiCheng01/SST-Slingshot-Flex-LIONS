
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
 @file main.cpp

 HPCG routine
 */

// Main routine of a program that calls the HPCG conjugate gradient
// solver to solve the problem, and then prints results.

#ifndef HPCG_NO_MPI
#include <mpi.h>
#endif

#include <fstream>
#include <iostream>
#include <cstdlib>
#ifdef HPCG_DETAILED_DEBUG
using std::cin;
#endif
using std::endl;

#include <vector>

#include "hpcg.hpp"

#include "CheckAspectRatio.hpp"
#include "GenerateGeometry.hpp"
#include "GenerateProblem.hpp"
#include "GenerateCoarseProblem.hpp"
#include "SetupHalo.hpp"
#include "CheckProblem.hpp"
#include "ExchangeHalo.hpp"
#include "OptimizeProblem.hpp"
#include "WriteProblem.hpp"
#include "ReportResults.hpp"
#include "mytimer.hpp"
#include "ComputeSPMV_ref.hpp"
#include "ComputeMG_ref.hpp"
#include "ComputeResidual.hpp"
#include "CG.hpp"
#include "CG_ref.hpp"
#include "Geometry.hpp"
#include "SparseMatrix.hpp"
#include "Vector.hpp"
#include "CGData.hpp"
#include "TestCG.hpp"
#include "TestSymmetry.hpp"
#include "TestNorms.hpp"

#include <sys/time.h>

/*!
  Main driver program: Construct synthetic problem, run V&V tests, compute benchmark parameters, run benchmark, report results.

  @param[in]  argc Standard argument count.  Should equal 1 (no arguments passed in) or 4 (nx, ny, nz passed in)
  @param[in]  argv Standard argument array.  If argc==1, argv is unused.  If argc==4, argv[1], argv[2], argv[3] will be interpreted as nx, ny, nz, resp.

  @return Returns zero on success and a non-zero value otherwise.

*/
int main(int argc, char * argv[]) {
  HPCG_Params params;

#ifndef HPCG_NO_MPI
  MPI_Init(&argc, &argv);
#else
  params.comm_size = 1;
  params.comm_rank = 0;
#endif

  timeval t_start; gettimeofday(&t_start, NULL);


  HPCG_Init(&argc, &argv, params);

  // Check if QuickPath option is enabled.
  // If the running time is set to zero, we minimize all paths through the program
  bool quickPath = (params.runningTime==0);

  int size = params.comm_size, rank = params.comm_rank; // Number of MPI processes, My process ID

#ifdef HPCG_DETAILED_DEBUG
  if (size < 100 && rank==0) HPCG_fout << "Process "<<rank<<" of "<<size<<" is alive with " << params.numThreads << " threads." <<endl;

  if (rank==0) {
    char c;
    std::cout << "Press key to continue"<< std::endl;
    std::cin.get(c);
  }
#ifndef HPCG_NO_MPI
  MPI_Barrier(MPI_COMM_WORLD);
#endif
#endif

  local_int_t nx,ny,nz;
  nx = (local_int_t)params.nx;
  ny = (local_int_t)params.ny;
  nz = (local_int_t)params.nz;
  int ierr = 0;  // Used to check return codes on function calls

  ierr = CheckAspectRatio(0.125, nx, ny, nz, "local problem", rank==0);
  if (ierr)
    return ierr;

  /////////////////////////
  // Problem setup Phase //
  /////////////////////////

#ifdef HPCG_DEBUG
  double t1 = mytimer();
#endif

  std::cout << "Test time main 00 in rank: " << rank << " at " << mytimer() << std::endl;

  // Construct the geometry and linear system
  Geometry * geom = new Geometry;
  GenerateGeometry(size, rank, params.numThreads, nx, ny, nz, geom);

  std::cout << "Test time main 01 in rank: " << rank << " at " << mytimer() << std::endl;
  ierr = CheckAspectRatio(0.125, geom->npx, geom->npy, geom->npz, "process grid", rank==0);
  if (ierr)
    return ierr;

  // Use this array for collecting timing information
  std::vector< double > times(10,0.0);

  double setup_time = mytimer();

  if (params.comm_rank == 0){
    printf("Running local grid nx=%d ny=%d nz=%d\n",
    params.nx, params.ny, params.nz);
  }

  std::cout << "Test time main 02 in rank: " << rank << " at " << mytimer() << std::endl;
  SparseMatrix A;
  InitializeSparseMatrix(A, geom);

  std::cout << "Test time main 03 in rank: " << rank << " at " << mytimer() << std::endl;
  Vector b, x, xexact;
  GenerateProblem(A, &b, &x, &xexact);
  std::cout << "Test time main 04 in rank: " << rank << " at " << mytimer() << std::endl;
  SetupHalo(A);
  std::cout << "Test time main 05 in rank: " << rank << " at " << mytimer() << std::endl;
  int numberOfMgLevels = 4; // Number of levels including first
  SparseMatrix * curLevelMatrix = &A;
  for (int level = 1; level< numberOfMgLevels; ++level) {
	  GenerateCoarseProblem(*curLevelMatrix);
	  curLevelMatrix = curLevelMatrix->Ac; // Make the just-constructed coarse grid the next level
  }

  std::cout << "Test time main 06 in rank: " << rank << " at " << mytimer() << std::endl;
  setup_time = mytimer() - setup_time; // Capture total time of setup
  times[9] = setup_time; // Save it for reporting

  curLevelMatrix = &A;
  Vector * curb = &b;
  Vector * curx = &x;
  Vector * curxexact = &xexact;
  for (int level = 0; level< numberOfMgLevels; ++level) {
     CheckProblem(*curLevelMatrix, curb, curx, curxexact);
     curLevelMatrix = curLevelMatrix->Ac; // Make the nextcoarse grid the next level
     curb = 0; // No vectors after the top level
     curx = 0;
     curxexact = 0;
  }

  std::cout << "Test time main 07 in rank: " << rank << " at " << mytimer() << std::endl;

  CGData data;
  InitializeSparseCGData(A, data);


  std::cout << "Test time main 08 in rank: " << rank << " at " << mytimer() << std::endl;

  ////////////////////////////////////
  // Reference SpMV+MG Timing Phase //
  ////////////////////////////////////

  // Call Reference SpMV and MG. Compute Optimization time as ratio of times in these routines

  local_int_t nrow = A.localNumberOfRows;
  local_int_t ncol = A.localNumberOfColumns;

  std::cout << "Test time main 09 in rank: " << rank << " at " << mytimer() << std::endl;
  Vector x_overlap, b_computed;
  InitializeVector(x_overlap, ncol); // Overlapped copy of x vector
  InitializeVector(b_computed, nrow); // Computed RHS vector


  std::cout << "Test time main 10 in rank: " << rank << " at " << mytimer() << std::endl;
  // Record execution time of reference SpMV and MG kernels for reporting times
  // First load vector with random values
  FillRandomVector(x_overlap);

  std::cout << "Test time main 11 in rank: " << rank << " at " << mytimer() << std::endl;
  int numberOfCalls = 10;
  if (quickPath) numberOfCalls = 1; //QuickPath means we do on one call of each block of repetitive code
  double t_begin = mytimer();
  for (int i=0; i< numberOfCalls; ++i) {

    std::cout << "Test time main 1100 in rank: " << rank << " at " << mytimer() << std::endl;
    ierr = ComputeSPMV_ref(A, x_overlap, b_computed); // b_computed = A*x_overlap
    if (ierr) HPCG_fout << "Error in call to SpMV: " << ierr << ".\n" << endl;
    std::cout << "Test time main 1101 in rank: " << rank << " at " << mytimer() << std::endl;
    ierr = ComputeMG_ref(A, b_computed, x_overlap); // b_computed = Minv*y_overlap
    if (ierr) HPCG_fout << "Error in call to MG: " << ierr << ".\n" << endl;
    std::cout << "Test time main 1102 in rank: " << rank << " at " << mytimer() << std::endl;
  }
  std::cout << "Test time main 12 in rank: " << rank << " at " << mytimer() << std::endl;
  times[8] = (mytimer() - t_begin)/((double) numberOfCalls);  // Total time divided by number of calls.
#ifdef HPCG_DEBUG
  if (rank==0) HPCG_fout << "Total SpMV+MG timing phase execution time in main (sec) = " << mytimer() - t1 << endl;
#endif

  ///////////////////////////////
  // Reference CG Timing Phase //
  ///////////////////////////////

#ifdef HPCG_DEBUG
  t1 = mytimer();
#endif
  int global_failure = 0; // assume all is well: no failures

  int niters = 0;
  int totalNiters_ref = 0;
  double normr = 0.0;
  double normr0 = 0.0;
  int refMaxIters = 5; //50 in regular code
  numberOfCalls = 1; // Only need to run the residual reduction analysis once

  // Compute the residual reduction for the natural ordering and reference kernels
  std::vector< double > ref_times(9,0.0);
  double tolerance = 0.0; // Set tolerance to zero to make all runs do maxIters iterations
  int err_count = 0;
  std::cout << "Test time main 13 in rank: " << rank << " at " << mytimer() << std::endl;
  for (int i=0; i< numberOfCalls; ++i) {
    ZeroVector(x);
    std::cout << "Test time main 1300 in rank: " << rank << " at " << mytimer() << std::endl;
    ierr = CG_ref( A, data, b, x, refMaxIters, tolerance, niters, normr, normr0, &ref_times[0], true);
    std::cout << "Test time main 1301 in rank: " << rank << " at " << mytimer() << std::endl;
    if (ierr) ++err_count; // count the number of errors in CG
    totalNiters_ref += niters;
  }
  std::cout << "Test time main 14 in rank: " << rank << " at " << mytimer() << std::endl;
  if (rank == 0 && err_count) HPCG_fout << err_count << " error(s) in call(s) to reference CG." << endl;
  double refTolerance = 0.0; //force to do max iters for SST: normr / normr0;

  // Call user-tunable set up function.
  double t7 = mytimer();
  std::cout << "Test time main 15 in rank: " << rank << " at " << mytimer() << std::endl;
  OptimizeProblem(A, data, b, x, xexact);
  std::cout << "Test time main 16 in rank: " << rank << " at " << mytimer() << std::endl;
  t7 = mytimer() - t7;
  times[7] = t7;
#ifdef HPCG_DEBUG
  if (rank==0) HPCG_fout << "Total problem setup time in main (sec) = " << mytimer() - t1 << endl;
#endif

#ifdef HPCG_DETAILED_DEBUG
  if (geom->size == 1) WriteProblem(*geom, A, b, x, xexact);
#endif


  //////////////////////////////
  // Validation Testing Phase //
  //////////////////////////////

#ifdef HPCG_DEBUG
  t1 = mytimer();
#endif
  TestCGData testcg_data;
  testcg_data.count_pass = testcg_data.count_fail = 0;
  std::cout << "Test time main 17 in rank: " << rank << " at " << mytimer() << std::endl;
  TestCG(A, data, b, x, testcg_data);

  std::cout << "Test time main 18 in rank: " << rank << " at " << mytimer() << std::endl;
  TestSymmetryData testsymmetry_data;
  TestSymmetry(A, b, xexact, testsymmetry_data);
  std::cout << "Test time main 19 in rank: " << rank << " at " << mytimer() << std::endl;

#ifdef HPCG_DEBUG
  if (rank==0) HPCG_fout << "Total validation (TestCG and TestSymmetry) execution time in main (sec) = " << mytimer() - t1 << endl;
#endif

#ifdef HPCG_DEBUG
  t1 = mytimer();
#endif

  //////////////////////////////
  // Optimized CG Setup Phase //
  //////////////////////////////

  niters = 0;
  normr = 0.0;
  normr0 = 0.0;
  err_count = 0;
  int tolerance_failures = 0;

  int optMaxIters = refMaxIters; //*10 in regular version
  int optNiters = refMaxIters;
  double opt_worst_time = 0.0;

  std::vector< double > opt_times(9,0.0);

  // Compute the residual reduction and residual count for the user ordering and optimized kernels.
  std::cout << "Test time main 20 in rank: " << rank << " at " << mytimer() << std::endl;
  for (int i=0; i< numberOfCalls; ++i) {
    ZeroVector(x); // start x at all zeros
    double last_cummulative_time = opt_times[0];
    ierr = CG( A, data, b, x, optMaxIters, refTolerance, niters, normr, normr0, &opt_times[0], true);
    if (ierr) ++err_count; // count the number of errors in CG
    if (normr / normr0 > refTolerance) ++tolerance_failures; // the number of failures to reduce residual

    // pick the largest number of iterations to guarantee convergence
    if (niters > optNiters) optNiters = niters;

    double current_time = opt_times[0] - last_cummulative_time;
    if (current_time > opt_worst_time) opt_worst_time = current_time;
  }

  std::cout << "Test time main 21 in rank: " << rank << " at " << mytimer() << std::endl;
#ifndef HPCG_NO_MPI
// Get the absolute worst time across all MPI ranks (time in CG can be different)
  double local_opt_worst_time = opt_worst_time;
  MPI_Allreduce(&local_opt_worst_time, &opt_worst_time, 1, MPI_DOUBLE, MPI_MAX, MPI_COMM_WORLD);
#endif

  std::cout << "Test time main 22 in rank: " << rank << " at " << mytimer() << std::endl;

  if (rank == 0 && err_count) HPCG_fout << err_count << " error(s) in call(s) to optimized CG." << endl;
  if (tolerance_failures) {
    global_failure = 1;
    if (rank == 0)
      HPCG_fout << "Failed to reduce the residual " << tolerance_failures << " times." << endl;
  }

  ///////////////////////////////
  // Optimized CG Timing Phase //
  ///////////////////////////////

  // Here we finally run the benchmark phase
  // The variable total_runtime is the target benchmark execution time in seconds

  double total_runtime = params.runningTime;
  int numberOfCgSets = 1; //int(total_runtime / opt_worst_time) + 1; // Run at least once, account for rounding

#ifdef HPCG_DEBUG
  if (rank==0) {
    HPCG_fout << "Projected running time: " << total_runtime << " seconds" << endl;
    HPCG_fout << "Number of CG sets: " << numberOfCgSets << endl;
  }
#endif

  /* This is the timed run for a specified amount of time. */

  optMaxIters = optNiters;
  double optTolerance = 0.0;  // Force optMaxIters iterations
  TestNormsData testnorms_data;
  testnorms_data.samples = numberOfCgSets;
#pragma sst new
  testnorms_data.values = new double[numberOfCgSets];

  std::cout << "Test time main 23 in rank: " << rank << " at " << mytimer() << std::endl;
  for (int i=0; i< numberOfCgSets; ++i) {
    ZeroVector(x); // Zero out x
    ierr = CG( A, data, b, x, optMaxIters, optTolerance, niters, normr, normr0, &times[0], true);
    if (ierr) HPCG_fout << "Error in call to CG: " << ierr << ".\n" << endl;
    if (rank==0) HPCG_fout << "Call [" << i << "] Scaled Residual [" << normr/normr0 << "]" << endl;
#pragma sst delete
    testnorms_data.values[i] = normr/normr0; // Record scaled residual from this run
  }

  std::cout << "Test time main 24 in rank: " << rank << " at " << mytimer() << std::endl;
  // Compute difference between known exact solution and computed solution
  // All processors are needed here.
#ifdef HPCG_DEBUG
  double residual = 0;
  ierr = ComputeResidual(A.localNumberOfRows, x, xexact, residual);
  if (ierr) HPCG_fout << "Error in call to compute_residual: " << ierr << ".\n" << endl;
  if (rank==0) HPCG_fout << "Difference between computed and exact  = " << residual << ".\n" << endl;
#endif

  std::cout << "Test time main 25 in rank: " << rank << " at " << mytimer() << std::endl;
  // Test Norm Results
  ierr = TestNorms(testnorms_data);
  std::cout << "Test time main 26 in rank: " << rank << " at " << mytimer() << std::endl;

  ////////////////////
  // Report Results //
  ////////////////////

  // Report results to YAML file
  ReportResults(A, numberOfMgLevels, numberOfCgSets, refMaxIters, optMaxIters, &times[0], testcg_data, testsymmetry_data, testnorms_data, global_failure, quickPath);
  std::cout << "Test time main 27 in rank: " << rank << " at " << mytimer() << std::endl;

  // Clean up
  DeleteMatrix(A); // This delete will recursively delete all coarse grid data
  DeleteCGData(data);
  DeleteVector(x);
  DeleteVector(b);
  DeleteVector(xexact);
  DeleteVector(x_overlap);
  DeleteVector(b_computed);
  delete [] testnorms_data.values;



  HPCG_Finalize();

  timeval t_stop; gettimeofday(&t_stop, NULL);
  double delta_t = (t_stop.tv_sec - t_start.tv_sec) + 1e-6*(t_stop.tv_usec - t_start.tv_usec);
  if (params.comm_rank == 0){
    printf("Total runtime %12.8fs\n", delta_t);
  }

  // Finish up
#ifndef HPCG_NO_MPI
  MPI_Finalize();
#endif
  return 0;
}
