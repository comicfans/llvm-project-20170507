// RUN: %clang_cc1 -triple x86_64-unknown-unknown -fsyntax-only -Wcast-qual -verify %s

#include <stdint.h>

void foo() {
  const char * const ptr = 0;
  const char * const *ptrptr = 0;
  char *y = (char *)ptr;	// expected-warning {{cast from 'const char *' to 'char *' drops const qualifier}}
  char **y1 = (char **)ptrptr;	// expected-warning {{cast from 'const char *const' to 'char *' drops const qualifier}}
  const char **y2 = (const char **)ptrptr;	// expected-warning {{cast from 'const char *const *' to 'const char **' drops const qualifier}}

  char *z = (char *)(uintptr_t)(const void *)ptr;	// no warning
  char *z1 = (char *)(const void *)ptr;	// expected-warning {{cast from 'const void *' to 'char *' drops const qualifier}}

  volatile char *vol = 0;
  char *vol2 = (char *)vol; // expected-warning {{cast from 'volatile char *' to 'char *' drops volatile qualifier}}
  const volatile char *volc = 0;
  char *volc2 = (char *)volc; // expected-warning {{cast from 'const volatile char *' to 'char *' drops const and volatile qualifiers}}

  int **intptrptr;
  const int **intptrptrc = (const int **)intptrptr; // expected-warning {{cast from 'int **' to 'const int **' must have all intermediate pointers const qualified}}
  volatile int **intptrptrv = (volatile int **)intptrptr; // expected-warning {{cast from 'int **' to 'volatile int **' must have all intermediate pointers const qualified}}

  int *intptr;
  const int *intptrc = (const int *)intptr;    // no warning

  const char **charptrptrc;
  char **charptrptr = (char **)charptrptrc; // expected-warning {{cast from 'const char *' to 'char *' drops const qualifier}}
}
