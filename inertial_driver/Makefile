CC?= gcc

TARGET=yesense_main

OBJS=yesense_main.o analysis_data.o

CFLAGS+=-c -Wall -g -O2

$(TARGET):$(OBJS)
	$(CC) $^ -o $@  

%.o:%.c                  
	$(CC) $^ $(CFLAGS) -o $@

clean:
	$(RM) *.o $(TARGET) -r
