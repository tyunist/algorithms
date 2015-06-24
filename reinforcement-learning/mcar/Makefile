#.KEEP_STATE:
#.SILENT:
EXEC	= xcar
EXEC2	= car
OFILES	= krandom.o modified-graphics.o mc.o display.o cmac.o qlearn-cmac-xwin.o
OFILES2	= krandom.o mc.o cmac.o qlearn-cmac.o

CC	= gcc

LIBS	=	-lm -lX11 -lsocket -lnsl
LIBS2	=	-lm 
#INCLUDE = 	-I/usr/openwin/share/include/X11/
# ty modified
INCLUDE = 	-I/usr/include/X11/


$(EXEC)	: $(OFILES)
	echo "linking $<"
	$(CC) -O3 -o $(EXEC) $(OFILES)  $(LIBS) $(INCLUDE)

$(EXEC2)	: $(OFILES2)
	echo "linking $<"
	$(CC) -O3 -o $(EXEC2) $(OFILES2)  $(LIBS2)

	echo -n "make finished at "; date

krandom.o : krandom.cpp krandom.h
	$(CC) -c krandom.cpp 

modified-graphics.o : modified-graphics.cpp graphics.h
	$(CC) -c modified-graphics.cpp $(INCLUDE)

mc.o : mc.cpp mc.h
	$(CC) -c mc.cpp

cmac.o : cmac.cpp cmac.h 
	$(CC) -c cmac.cpp

display.o : display.cpp display.h modified-graphics.o
	$(CC) -c display.cpp $(INCLUDE)

qlearn-cmac-xwin.o : modified-graphics.cpp krandom.o cmac.o display.o mc.o qlearn-cmac-xwin.cpp
	$(CC) -c qlearn-cmac-xwin.cpp $(INCLUDE)

qlearn-cmac.o : qlearn-cmac.cpp krandom.o cmac.o mc.o 
	$(CC) -c qlearn-cmac.cpp


# Clean up after making. 
clean:
	echo "removing" *.o
	$(RM) *.o
	if [ -d /tmp/$(USER)tmp ]; \
	 then echo "clearing /tmp/$(USER)tmp"; $(MAKE) tclean; fi


# Time your make.
time:
	/usr/bin/time $(MAKE)




