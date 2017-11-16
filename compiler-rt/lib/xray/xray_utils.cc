//===-- xray_utils.cc -------------------------------------------*- C++ -*-===//
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
//===----------------------------------------------------------------------===//
#include "xray_utils.h"

#include "sanitizer_common/sanitizer_common.h"
#include "xray_defs.h"
#include "xray_flags.h"
#include <stdlib.h>
#include <cstdio>
#include <errno.h>
#include <fcntl.h>
#include <iterator>
#include <sys/types.h>
#include <tuple>
#include <utility>
#ifndef _WIN32
#include <unistd.h>
#else
#include <fcntl.h>
#endif

namespace __xray {

void printToStdErr(const char *Buffer) XRAY_NEVER_INSTRUMENT {
  fprintf(stderr, "%s", Buffer);
}

#ifndef _WIN32
void retryingWriteAll(int Fd, char *Begin, char *End) XRAY_NEVER_INSTRUMENT {
  if (Begin == End)
    return;
  auto TotalBytes = std::distance(Begin, End);
  while (auto Written = write(Fd, Begin, TotalBytes)) {
    if (Written < 0) {
      if (errno == EINTR)
        continue; // Try again.
      Report("Failed to write; errno = %d\n", errno);
      return;
    }
    TotalBytes -= Written;
    if (TotalBytes == 0)
      break;
    Begin += Written;
  }
}
#else
void retryingWriteAll(FileDescription Fd, char *Begin, char *End) XRAY_NEVER_INSTRUMENT {
  if (Begin == End)
    return;
  auto TotalBytes = std::distance(Begin, End);
  DWORD Written=0;
  while (WriteFile(Fd, Begin, TotalBytes,&Written,nullptr)) {
    if (Written < 0) {
      if (errno == EINTR)
        continue; // Try again.
      Report("Failed to write; errno = %d\n", errno);
      return;
    }
    TotalBytes -= Written;
    if (TotalBytes == 0)
      break;
    Begin += Written;
  }
}
#endif

#ifndef _WIN32
std::pair<ssize_t, bool> retryingReadSome(FileDescription Fd, char *Begin,
                                          char *End) XRAY_NEVER_INSTRUMENT {
  auto BytesToRead = std::distance(Begin, End);
  ssize_t BytesRead;
  ssize_t TotalBytesRead = 0;
  while (BytesToRead && (BytesRead = read(Fd, Begin, BytesToRead))) {
    if (BytesRead == -1) {
      if (errno == EINTR)
        continue;
      Report("Read error; errno = %d\n", errno);
      return std::make_pair(TotalBytesRead, false);
    }

    TotalBytesRead += BytesRead;
    BytesToRead -= BytesRead;
    Begin += BytesRead;
  }
  return std::make_pair(TotalBytesRead, true);
}
#else
std::pair<ssize_t, bool> retryingReadSome(FileDescription Fd, char *Begin,
                                          char *End) XRAY_NEVER_INSTRUMENT {
  auto BytesToRead = std::distance(Begin, End);
  DWORD BytesRead;
  ssize_t TotalBytesRead = 0;
  while (BytesToRead ) {
	BOOL ok=ReadFile(Fd,Begin,BytesToRead,&BytesRead,nullptr);

    if (!ok) {
      Report("Read error; errno = %d\n", errno);
      return std::make_pair(TotalBytesRead, false);
    }

    TotalBytesRead += BytesRead;
    BytesToRead -= BytesRead;
    Begin += BytesRead;
  }
  return std::make_pair(TotalBytesRead, true);
}
#endif

bool readValueFromFile(const char *Filename,
                       long long *Value) XRAY_NEVER_INSTRUMENT {

	FileDescription Fd = 
#ifndef _WIN32
   _open(Filename, O_RDONLY );
#else
  CreateFileA(Filename, GENERIC_READ,0,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
#endif
  if (Fd == FileDescription(-1))
    return false;
  static constexpr size_t BufSize = 256;
  char Line[BufSize] = {};
  ssize_t BytesRead;
  bool Success;
  std::tie(BytesRead, Success) = retryingReadSome(Fd, Line, Line + BufSize);
  if (!Success)
    return false;
#ifndef _WIN32
  _close(Fd);
#else
  CloseHandle(Fd);
#endif
  char *End = nullptr;
  long long Tmp = internal_simple_strtoll(Line, &End, 10);
  bool Result = false;
  if (Line[0] != '\0' && (*End == '\n' || *End == '\0')) {
    *Value = Tmp;
    Result = true;
  }
  return Result;
}

FileDescription getLogFD() XRAY_NEVER_INSTRUMENT {
  // Open a temporary file once for the log.
  static char TmpFilename[256] = {};
  static char TmpWildcardPattern[] = "XXXXXX";
  auto Argv = GetArgv();
  const char *Progname = Argv[0] == nullptr ? "(unknown)" : Argv[0];
  const char *LastSlash = internal_strrchr(Progname, '/');

  if (LastSlash != nullptr)
    Progname = LastSlash + 1;

  const int HalfLength = sizeof(TmpFilename) / 2 - sizeof(TmpWildcardPattern);
  int NeededLength = internal_snprintf(
      TmpFilename, sizeof(TmpFilename), "%.*s%.*s.%s", HalfLength,
      flags()->xray_logfile_base, HalfLength, Progname, TmpWildcardPattern);
  if (NeededLength > int(sizeof(TmpFilename))) {
    Report("XRay log file name too long (%d): %s\n", NeededLength, TmpFilename);
    return INVALID_HANDLE_VALUE;
  }
#ifndef _WIN32
  FileDescription Fd = mkstemp(TmpFilename);
#else

  char ResultName[MAX_PATH];

  GetTempFileNameA(".", // directory for tmp files
                              TmpFilename,     // temp file name prefix 
                              0,                // create unique name 
                              ResultName); 
  FileDescription Fd = CreateFileA(ResultName,GENERIC_READ,0,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);

#endif
  if (Fd == FileDescription(-1)) {
    Report("XRay: Failed opening temporary file '%s'; not logging events.\n",
           TmpFilename);
    return FileDescription(-1);
  }
  Report("XRay: Log file in '%s'\n", TmpFilename);

  return Fd;
}

} // namespace __xray
