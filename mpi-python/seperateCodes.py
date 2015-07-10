#seperateCodes.py
from mpi4py import MPI
rank = MPI.COMM_WORLD.Get_rank()

a = 6.0
b = 3.0
if rank == 0:
	print 'process rank 0: ', a + b
if rank == 1:
	print 'process rank 1:', a * b
if rank == 2:
	print 'process rank 2:',  max(a,b)
