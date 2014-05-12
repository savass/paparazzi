# Quiet compilation
Q=@

all: ant_tr

ant_tr: main.c
	gcc -g -O2 -Wall -export-dynamic `pkg-config libxml-2.0 ivy-glib --cflags --libs gtk+-3.0` -o $@ $^ -lm -lz `pkg-config libxml-2.0 ivy-glib libpcre --libs gtk+-3.0` -D USE_ORION

clean:
	$(Q)rm -f ant_tr


#gcc -g -O2 -Wall `pkg-config libxml-2.0 ivy-glib --cflags` -o $@ $^ -lm -lz `pkg-config libxml-2.0 ivy-glib libpcre --libs gtk+-3.0`
#gcc -Wall -g -o "%e" "%f" -export-dynamic `pkg-config --cflags --libs gtk+-3.0` -s -lglibivy -I/usr/include/libxml2 -L/usr/lib -lxml2 -lz -lpthread -lm
