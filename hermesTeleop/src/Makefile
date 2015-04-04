CC = g++
CFLAGS = -I. -L.
TARGET = HelloEposCmd
LIBS = -lEposCmd -lftd2xx

all: $(TARGET)

$(TARGET): $(TARGET).cpp
	$(CC) -o $(TARGET) $(TARGET).cpp $(CFLAGS) $(LIBS)

clean:
	$(RM) $(TARGET)
