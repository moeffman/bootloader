#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

void _exit(int status) {
    while (1) {
        // Infinite loop to hang
    }
}

caddr_t _sbrk(/*int increment*/) {
    return (caddr_t)-1;  // Not implemented
}

int _write(/*int file, char *ptr, int len*/) {
    return -1;  // Not implemented
}

int _close(/*int file*/) {
    return -1;  // Not implemented
}

int _lseek(/*int file, int ptr, int dir*/) {
    return -1;  // Not implemented
}

int _read(/*int file, char *ptr, int len*/) {
    return -1;  // Not implemented
}

int _fstat(/*int file, */struct stat *st) {
    st->st_mode = S_IFCHR;  // Character device
    return 0;
}

int _isatty(/*int file*/) {
    return 1;
}

