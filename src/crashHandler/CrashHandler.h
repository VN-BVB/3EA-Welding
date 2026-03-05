#ifndef CRASHHANDLER_H
#define CRASHHANDLER_H

#include <Windows.h>

#include <string>

class CrashHandler {
public:
    static void Init(const std::wstring& dumpDir = L"crash_logs");

private:
    static LONG WINAPI HandleException(EXCEPTION_POINTERS* pException);
    static void WriteMiniDump(EXCEPTION_POINTERS* pException, const std::wstring& dmpPath);
    static void WriteCrashInfo(EXCEPTION_POINTERS* pException, const std::wstring& txtPath);
    static std::wstring GetExceptionDescription(DWORD code);
};

#endif  // CRASHHANDLER_H
