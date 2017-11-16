//===-- xray_utils.h --------------------------------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file is a part of XRay, a dynamic runtime instrumentation system.
//
// Some shared utilities for the XRay runtime implementation.
//
//===----------------------------------------------------------------------===//
#ifndef XRAY_UTILS_H
#define XRAY_UTILS_H

#include <sys/types.h>
#include <utility>

#ifdef _WIN32
typedef __int64 ssize_t;
#include <windows.h>
#endif

namespace __xray {

#ifndef _WIN32
	typedef int FileDescription;
#else
	typedef HANDLE FileDescription;
#endif

// Default implementation of the reporting interface for sanitizer errors.
void printToStdErr(const char *Buffer);

// EINTR-safe write routine, provided a file descriptor and a character range.
void retryingWriteAll(FileDescription Fd, char *Begin, char *End);

// Reads a long long value from a provided file.
bool readValueFromFile(const char *Filename, long long *Value);

// EINTR-safe read routine, providing a file descriptor and a character range.
std::pair<ssize_t, bool> retryingReadSome(FileDescription Fd, char *Begin, char *End);

// EINTR-safe open routine, uses flag-provided values for initialising a log
// file.
FileDescription getLogFD();

} // namespace __xray

#endif // XRAY_UTILS_H
