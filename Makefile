CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -pedantic -g
LDFLAGS = -lm
SRC = src/main.c src/physics.c src/gnc.c
TARGET = aerognc

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)
