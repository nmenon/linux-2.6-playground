CC		= $(CROSS_COMPILE)gcc
BUILD_OUTPUT	:= $(PWD)
PREFIX		:= /usr
DESTDIR		:=

fspin : fspin.c
CFLAGS +=	-Wall

%: %.c
	@mkdir -p $(BUILD_OUTPUT)
	$(CC) $(CFLAGS) $< -o $(BUILD_OUTPUT)/$@ -lpthread

.PHONY : clean
clean :
	@rm -f $(BUILD_OUTPUT)/fspin

install : fspin
	install -d  $(DESTDIR)$(PREFIX)/bin
	install $(BUILD_OUTPUT)/fspin $(DESTDIR)$(PREFIX)/bin/fspin
	install -d  $(DESTDIR)$(PREFIX)/share/man/man1
	install fspin.1 $(DESTDIR)$(PREFIX)/share/man/man1
