//===----------------------------------------------------------------------===//
//
// ΚΚΚΚΚΚΚΚΚΚΚΚΚΚΚΚΚΚΚΚThe LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

// <string>

// template<> struct char_traits<wchar_t>

// static constexpr bool lt(char_type c1, char_type c2);

#include <string>
#include <cassert>

int main()
{
    wchar_t c = L'\0';
    assert(!std::char_traits<wchar_t>::lt(L'a', L'a'));
    assert( std::char_traits<wchar_t>::lt(L'A', L'a'));
}
