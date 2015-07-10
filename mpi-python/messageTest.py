#passRandomDraw.py
import numpy as np 
from mpi4py import MPI
comm = MPI.COMM_WORLD
rank = comm.Get_rank()

# Initialize
#
#
if rank == 1:
	print "Process", rank, "Sending...Continue"
	randum = np.array(['continue'])
	comm.isend(randum, dest=0)
	print "Process", rank, "receiving..."
	planner_buffer = comm.irecv()
	print "Process", rank, "received ", planner_buffer

if rank == 0:
	print "Process", rank, "Receiving..."
	simulator_buffer = comm.irecv()
	print "Process", rank, "received the string: ", simulator_buffer
	print "type of string:", type(simulator_buffer)
	print "Process", rank, "sending...1"
	randum = [0]
	comm.isend(randum, dest=1)

