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

ch5_ex5_1: ch5_ex5_1.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch5_ex5_2: ch5_ex5_2.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

pickcolor: pickcolor.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch2_ex2_3: ch2_ex2_3.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch2_ex2_4: ch2_ex2_4.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch2_ex2_5: ch2_ex2_5.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch2_ex2_6: ch2_ex2_6.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch2_ex2_7: ch2_ex2_7.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_3: ch3_ex3_3.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_4: ch3_ex3_4.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_5: ch3_ex3_5.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_9: ch3_ex3_9.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_11: ch3_ex3_11.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_12: ch3_ex3_12.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_13: ch3_ex3_13.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_14: ch3_ex3_14.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_15: ch3_ex3_15.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_17: ch3_ex3_17.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_19: ch3_ex3_19.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)

ch3_ex3_20: ch3_ex3_20.o 
			$(CC) -o $@ $(ODIR)/$< $(LIBS)
			
clean: 
		rm -f *.o $(ODIR)/*.o *~ core /*~
