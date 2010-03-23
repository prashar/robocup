IDIR =/usr/local/include/opencv
CC=g++
LDIR=/usr/local/lib
ODIR=obj
CFLAGS=-I$(IDIR) -L$(LDIR)
LIBS=-lm -lcv -lhighgui -lcvaux -lcxcore -ldc1394
DEPS= 

%.o: %.cpp $(DEPS)
			$(CC) -c $< -o $(ODIR)/$@ $(CFLAGS) 

hough: hough.o
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

objectDetector: objectDetector.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch6_ex6_1: ch6_ex6_1.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch8_ex2: ch8_ex2.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch8_ex8_2: ch8_ex8_2.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch8_ex8_3: ch8_ex8_3.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch5_ex5_1: ch5_ex5_1.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

houghex: houghex.o
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

houghex2: houghex2.o
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

clean: 
		rm -f *.o $(ODIR)/*.o *~ core /*~
