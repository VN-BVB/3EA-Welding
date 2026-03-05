#include "CrashHandler.h"

#include <DbgHelp.h>

#include <fstream>
#include <iomanip>
#include <sstream>

#pragma comment(lib, "Dbghelp.lib")

void CrashHandler::Init(const std::wstring& dumpDir) {
    // CreateDirectoryW(dumpDir.c_str(), NULL);  // 创建文件夹
    SetUnhandledExceptionFilter(HandleException);
    (void)dumpDir;
}

LONG WINAPI CrashHandler::HandleException(EXCEPTION_POINTERS* pException) {
    SYSTEMTIME st;
    GetLocalTime(&st);
    wchar_t name[256];
    swprintf(name, 256, L"crash_%04d%02d%02d_%02d%02d%02d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

    std::wstring dir = L"data/debug/";
    std::wstring dmpPath = dir + name + L".dmp";
    std::wstring txtPath = dir + name + L".txt";

    CreateDirectoryW(dir.c_str(), NULL);

    // WriteMiniDump(pException, dmpPath);
    WriteCrashInfo(pException, txtPath);

    return EXCEPTION_EXECUTE_HANDLER;
}

void CrashHandler::WriteMiniDump(EXCEPTION_POINTERS* pException, const std::wstring& dmpPath) {
    HANDLE hFile = CreateFileW(dmpPath.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hFile == INVALID_HANDLE_VALUE) return;

    MINIDUMP_EXCEPTION_INFORMATION mei;
    mei.ThreadId = GetCurrentThreadId();
    mei.ExceptionPointers = pException;
    mei.ClientPointers = FALSE;

    MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile, MiniDumpWithFullMemory, &mei, nullptr, nullptr);
    CloseHandle(hFile);
}

std::wstring CrashHandler::GetExceptionDescription(DWORD code) {
    if (code == EXCEPTION_ACCESS_VIOLATION)
        return L"Access Violation";
    else if (code == EXCEPTION_ARRAY_BOUNDS_EXCEEDED)
        return L"Array Bounds Exceeded";
    else if (code == EXCEPTION_BREAKPOINT)
        return L"Breakpoint";
    else if (code == EXCEPTION_DATATYPE_MISALIGNMENT)
        return L"Datatype Misalignment";
    else if (code == EXCEPTION_FLT_DIVIDE_BY_ZERO)
        return L"Float Divide by Zero";
    else if (code == EXCEPTION_INT_DIVIDE_BY_ZERO)
        return L"Integer Divide by Zero";
    else if (code == EXCEPTION_STACK_OVERFLOW)
        return L"Stack Overflow";
    else
        return L"Unknown Exception";
}

void CrashHandler::WriteCrashInfo(EXCEPTION_POINTERS* pException, const std::wstring& txtPath) {
    std::wofstream file(txtPath.c_str());
    if (!file.is_open()) return;

    SYSTEMTIME st;
    GetLocalTime(&st);

    file << L"Crash time: " << st.wYear << L"-" << std::setw(2) << std::setfill(L'0') << st.wMonth << L"-" << std::setw(2) << std::setfill(L'0')
         << st.wDay << L" " << std::setw(2) << std::setfill(L'0') << st.wHour << L":" << std::setw(2) << std::setfill(L'0') << st.wMinute << L":"
         << std::setw(2) << std::setfill(L'0') << st.wSecond << std::endl;

    DWORD code = pException->ExceptionRecord->ExceptionCode;
    void* address = pException->ExceptionRecord->ExceptionAddress;

    file << L"Exception code: 0x" << std::hex << code << std::endl;
    file << L"Exception type: " << GetExceptionDescription(code) << std::endl;
    file << L"Exception address: 0x" << std::hex << reinterpret_cast<uintptr_t>(address) << std::endl;
    file << L"Thread ID: " << std::dec << GetCurrentThreadId() << std::endl;

    HANDLE hProcess = GetCurrentProcess();
    SymInitialize(hProcess, NULL, TRUE);

    DWORD64 addr = reinterpret_cast<DWORD64>(address);
    char symbolBuffer[sizeof(SYMBOL_INFO) + MAX_SYM_NAME] = {};
    SYMBOL_INFO* symbol = reinterpret_cast<SYMBOL_INFO*>(symbolBuffer);
    symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
    symbol->MaxNameLen = MAX_SYM_NAME;

    if (SymFromAddr(hProcess, addr, NULL, symbol)) {
        file << L"\nCrash function: ";
        wchar_t wFuncName[1024];
        mbstowcs(wFuncName, symbol->Name, sizeof(wFuncName) / sizeof(wchar_t));
        file << wFuncName << std::endl;
    }

    IMAGEHLP_LINE64 line;
    ZeroMemory(&line, sizeof(line));
    line.SizeOfStruct = sizeof(IMAGEHLP_LINE64);
    DWORD displacement = 0;
    if (SymGetLineFromAddr64(hProcess, addr, &displacement, &line)) {
        file << L"Source file: ";
        wchar_t wFileName[MAX_PATH];
        mbstowcs(wFileName, line.FileName, MAX_PATH);
        file << wFileName << std::endl;
        file << L"Line: " << std::dec << line.LineNumber << std::endl;
    }

    file << L"\nCall Stack:\n";

    CONTEXT context = *pException->ContextRecord;
    STACKFRAME64 stack = {};
#ifdef _M_IX86
    DWORD imageType = IMAGE_FILE_MACHINE_I386;
    stack.AddrPC.Offset = context.Eip;
    stack.AddrPC.Mode = AddrModeFlat;
    stack.AddrFrame.Offset = context.Ebp;
    stack.AddrFrame.Mode = AddrModeFlat;
    stack.AddrStack.Offset = context.Esp;
    stack.AddrStack.Mode = AddrModeFlat;
#elif _M_X64
    DWORD imageType = IMAGE_FILE_MACHINE_AMD64;
    stack.AddrPC.Offset = context.Rip;
    stack.AddrPC.Mode = AddrModeFlat;
    stack.AddrFrame.Offset = context.Rsp;
    stack.AddrFrame.Mode = AddrModeFlat;
    stack.AddrStack.Offset = context.Rsp;
    stack.AddrStack.Mode = AddrModeFlat;
#endif

    for (int frame = 0; frame < 50; ++frame) {
        if (!StackWalk64(imageType, hProcess, GetCurrentThread(), &stack, &context, nullptr, SymFunctionTableAccess64, SymGetModuleBase64, nullptr))
            break;

        if (stack.AddrPC.Offset == 0) break;

        DWORD64 addr = stack.AddrPC.Offset;
        if (SymFromAddr(hProcess, addr, NULL, symbol)) {
            wchar_t wName[1024];
            mbstowcs(wName, symbol->Name, sizeof(wName) / sizeof(wchar_t));
            file << L"#" << frame << L"  " << wName;
        } else {
            file << L"#" << frame << L"  [Unknown Symbol]";
        }

        if (SymGetLineFromAddr64(hProcess, addr, &displacement, &line)) {
            wchar_t wFileName[MAX_PATH];
            mbstowcs(wFileName, line.FileName, MAX_PATH);
            file << L" [" << wFileName << L":" << line.LineNumber << L"]\n";
        } else {
            file << L"\n";
        }
    }

    SymCleanup(hProcess);
    file.close();
}
