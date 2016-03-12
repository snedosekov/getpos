# makefile for rtkrcv

BINDIR = /usr/local/bin
SRC    = ./src
RCVSRC    = ./src/rcv
ODIR=./obj

# for beagleboard
#CTARGET= -mfpu=neon -mfloat-abi=softfp -ffast-math
CTARGET= -DENAGLO -DENAQZS -DENACMP -DNFREQ=3

CFLAGS = -Wall -O3 -ansi -pedantic -Wno-unused-but-set-variable -I$(SRC) -I.. -DTRACE $(CTARGET) -g
LDLIBS  = -lm -lrt -lpthread

_OBJ = getpos.o vt.o rtkcmn.o rtksvr.o rtkpos.o geoid.o solution.o lambda.o \
sbas.o stream.o rcvraw.o rtcm.o preceph.o options.o pntpos.o ppp.o ppp_ar.o \
ephemeris.o rinex.o ionex.o rtcm2.o rtcm3.o rtcm3e.o qzslex.o \

_RCVOBJ = novatel.o ublox.o ss2.o crescent.o skytraq.o binex.o gw10.o javad.o nvs.o rt17.o

OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
RCVOBJ = $(patsubst %,$(ODIR)/%,$(_RCVOBJ))

all        : getpos
getpos     : $(ODIR) $(OBJ) $(RCVOBJ)
	$(CC) $(OBJ) $(RCVOBJ) -Wall -o $@ $(LDLIBS)
$(ODIR)/%.o: $(SRC)/%.c $(SRC)/rtklib.h
	$(CC) -c $(CFLAGS) -o $@ -c $<
$(ODIR)/%.o: $(RCVSRC)/%.c $(SRC)/rtklib.h
	$(CC) -c $(CFLAGS) -o $@ -c $<
$(ODIR)/%.o   : $(SRC)/getpos/%.c
	$(CC) -c $(CFLAGS) -o $@ -c $< 
$(ODIR):
	mkdir $(ODIR)

clean:
	rm -f getpos *.nav $(ODIR)/*.o *.out *.trace

