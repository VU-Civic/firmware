#ifndef __BOARD_REVISIONS_HEADER_H__
#define __BOARD_REVISIONS_HEADER_H__

#ifndef BOARD_REV
#error "The BOARD_REV macro is undefined. Please set it to a proper board revision before compiling."
#endif

#define REV_A                    0x01
#define REV_B                    0x02

#define REV(rev)                 REV_ ## rev
#define REVISION(rev)            REV(rev)
#define REV_ID                   REVISION(BOARD_REV)

#endif  // #ifndef __BOARD_REVISIONS_HEADER_H__
