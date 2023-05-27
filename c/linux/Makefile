CC := gcc
CFLAGS := -Wall -Wextra -Werror

SRCDIR := .
INCDIR := -I. -I../../arduino -I../.. -I../inc

LIB_SRC := $(wildcard $(SRCDIR)/*.c)
OBJ := $(filter-out $(SRCDIR)/test.o, $(LIB_SRC:.c=.o))

TARGET := test
STATIC_LIB := libui2c.a
SHARED_LIB := libui2c.so

.PHONY: all clean test static shared

all: test

test: $(SRCDIR)/test.o $(STATIC_LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L. -lui2c -o $(TARGET)

static: $(STATIC_LIB)

shared: $(SHARED_LIB)

$(STATIC_LIB): $(OBJ)
	ar rcs $@ $^

$(SHARED_LIB): $(OBJ)
	$(CC) -shared  -o $@ $^

$(SRCDIR)/test.o: $(SRCDIR)/test.c
	$(CC) $(CFLAGS) $(INCDIR) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) $(INCDIR) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET) $(STATIC_LIB) $(SHARED_LIB) $(SRCDIR)/test.o
