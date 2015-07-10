# send_array
from mpi4py import MPI
import numpy 
import time
import myTime  
comm = MPI.COMM_WORLD
size = comm.Get_size()
rank = comm.Get_rank()

a_size = 10

send_buffer = (rank+1)*numpy.arange(5, dtype=numpy.float64)
recv_buffer = (rank+2)*numpy.arange(2, dtype=numpy.float64)
if rank == 0:
	count = 1 
	timer =  myTime.Time()
	while True: 
		print 'Now:', timer.stop(),  '- count: ', count, '- on task',rank,'before sending:    data = ',send_buffer
		req=comm.Isend(send_buffer,1,11)
		print 'on task',rank,'before recv:   data = ',recv_buffer
		response=comm.Irecv(recv_buffer,source=1,tag=11)
		re = False
		while re == False :
		   re=MPI.Request.Test(response)
		print 'test result',re
		re=MPI.Request.Wait(response)
		# print 'wait result',re
		print 'on task',rank,'after recv:    data = ',recv_buffer
		time.sleep(1)
		count = count + 1
		
elif rank == 1:
	coun = 1 # while coun increases double after each iteration? 
	timer = myTime.Time()
	while True: 
		print 'Now:', timer.stop(),   '- coun: ', coun, '- on task',rank,'before recv:   data = ',send_buffer
		req=comm.Irecv(send_buffer,source=0,tag=11)
		re = False
		while re == False :
		   re=MPI.Request.Test(req)
		print 'test result',re
		re=MPI.Request.Wait(req)
		# print 'wait result',re
		print 'on task',rank,'after recv:    sent= ',send_buffer
		print 'on task',rank,'before sending:    recv = ',recv_buffer
		req=comm.Isend(recv_buffer,0,11)
		time.sleep(2)
		coun = coun + 1