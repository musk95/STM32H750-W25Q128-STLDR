/*
 * Loader_Src.h
 *
 *  Created on: Jan 22, 2020
 *      Author: Kotetsu Yamamoto
 *      Copyright [Kotetsu Yamamoto]
 *
MIT License
Copyright (c) 2020 Kotetsu Yamamoto
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *
 */
#ifndef INC_LOADER_SRC_H
#define INC_LOADER_SRC_H

#ifdef __ICCARM__                 //IAR
#define KeepInCompilation __root
#elif __CC_ARM                    //MDK-ARM
#define KeepInCompilation __attribute__((used))
#else // TASKING                  //TrueStudio
#define KeepInCompilation __attribute__((used))
#endif

int Init(uint8_t MemMappedMode);
int WriteEnable (void);
KeepInCompilation int Write (uint32_t Address, uint32_t Size, uint8_t* buffer);
KeepInCompilation int SectorErase (uint32_t EraseStartAddress ,uint32_t EraseEndAddress);
int MassErase (void);
int Read (uint32_t Address, uint32_t Size, uint8_t* Buffer);
KeepInCompilation int Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size);

#endif // INC_LOADER_SRC_H
