//===-Caching.cpp - LLVM Link Time Optimizer Cache Handling ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the Caching for ThinLTO.
//
//===----------------------------------------------------------------------===//

#include "llvm/LTO/Caching.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/Errc.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/Process.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;
using namespace llvm::lto;

Expected<NativeObjectCache> lto::localCache(StringRef CacheDirectoryPath,
                                            AddBufferFn AddBuffer) {
  if (std::error_code EC = sys::fs::create_directories(CacheDirectoryPath))
    return errorCodeToError(EC);

  return [=](unsigned Task, StringRef Key) -> AddStreamFn {
    // This choice of file name allows the cache to be pruned (see pruneCache()
    // in include/llvm/Support/CachePruning.h).
    SmallString<64> EntryPath;
    sys::path::append(EntryPath, CacheDirectoryPath, "llvmcache-" + Key);
    // First, see if we have a cache hit.
    ErrorOr<std::unique_ptr<MemoryBuffer>> MBOrErr =
        MemoryBuffer::getFile(EntryPath);
    if (MBOrErr) {
      AddBuffer(Task, std::move(*MBOrErr), EntryPath);
      return AddStreamFn();
    }

    if (MBOrErr.getError() != errc::no_such_file_or_directory)
      report_fatal_error(Twine("Failed to open cache file ") + EntryPath +
                         ": " + MBOrErr.getError().message() + "\n");

    // This native object stream is responsible for commiting the resulting
    // file to the cache and calling AddBuffer to add it to the link.
    struct CacheStream : NativeObjectStream {
      AddBufferFn AddBuffer;
      sys::fs::TempFile TempFile;
      std::string EntryPath;
      unsigned Task;

      CacheStream(std::unique_ptr<raw_pwrite_stream> OS, AddBufferFn AddBuffer,
                  sys::fs::TempFile TempFile, std::string EntryPath,
                  unsigned Task)
          : NativeObjectStream(std::move(OS)), AddBuffer(std::move(AddBuffer)),
            TempFile(std::move(TempFile)), EntryPath(std::move(EntryPath)),
            Task(Task) {}

      ~CacheStream() {
        // Make sure the stream is closed before committing it.
        OS.reset();

        // Open the file first to avoid racing with a cache pruner.
        ErrorOr<std::unique_ptr<MemoryBuffer>> MBOrErr =
            MemoryBuffer::getOpenFile(TempFile.FD, TempFile.TmpName,
                                      /*FileSize*/ -1,
                                      /*RequiresNullTerminator*/ false);
        if (!MBOrErr)
          report_fatal_error(Twine("Failed to open new cache file ") +
                             TempFile.TmpName + ": " +
                             MBOrErr.getError().message() + "\n");

        // This is atomic on POSIX systems.
        // On Windows, it can fail with permission denied if the destination
        // file already exists. Since the existing file should be semantically
        // equivalent to the one we are trying to write, we give AddBuffer
        // a copy of the bytes we wrote in that case. We do this instead of
        // just using the existing file, because the pruner might delete the
        // file before we get a chance to use it.
        Error E = TempFile.keep(EntryPath);
        E = handleErrors(std::move(E), [&](const ECError &E) -> Error {
          std::error_code EC = E.convertToErrorCode();
          if (EC != errc::permission_denied)
            return errorCodeToError(EC);

          auto MBCopy = MemoryBuffer::getMemBufferCopy((*MBOrErr)->getBuffer(),
                                                       EntryPath);
          MBOrErr = std::move(MBCopy);

          // FIXME: should we consume the discard error?
          consumeError(TempFile.discard());

          return Error::success();
        });

        if (E)
          report_fatal_error(Twine("Failed to rename temporary file ") +
                             TempFile.TmpName + " to " + EntryPath + ": " +
                             toString(std::move(E)) + "\n");

        AddBuffer(Task, std::move(*MBOrErr), EntryPath);
      }
    };

    return [=](size_t Task) -> std::unique_ptr<NativeObjectStream> {
      // Write to a temporary to avoid race condition
      SmallString<64> TempFilenameModel;
      sys::path::append(TempFilenameModel, CacheDirectoryPath, "Thin-%%%%%%.tmp.o");
      Expected<sys::fs::TempFile> Temp = sys::fs::TempFile::create(
          TempFilenameModel, sys::fs::owner_read | sys::fs::owner_write);
      if (!Temp) {
        errs() << "Error: " << toString(Temp.takeError()) << "\n";
        report_fatal_error("ThinLTO: Can't get a temporary file");
      }

      // This CacheStream will move the temporary file into the cache when done.
      return llvm::make_unique<CacheStream>(
          llvm::make_unique<raw_fd_ostream>(Temp->FD, /* ShouldClose */ false),
          AddBuffer, std::move(*Temp), EntryPath.str(), Task);
    };
  };
}
