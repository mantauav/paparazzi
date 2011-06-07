/** \file
    A Simple Unit Test Framework Macros and Definitions
    $Id$
    
    ***Warning*** Aurora Flight Sciences, Inc. Proprietary Material
    Author: Jim Francis
    Copyright (C) 2010 by Aurora Flight Sciences, Inc. All Rights Reserved.
*/
#ifndef unittest_h
#define unittest_h

#define TESTCASE(name) { (char*)#name, name, 0 }

#define ASSERT_TRUE(cond) if ( cond ) { } else {	\
    failureFilename = (char*)__FILE__;			\
    failureLine = __LINE__;				\
    failureMessage = (char*)#cond;			\
    return 0; }

#define ASSERT_FALSE(cond) if ( cond ) {	\
    failureFilename = (char*)__FILE__;		\
    failureLine = __LINE__;			\
    failureMessage = (char*)#cond;		\
    return 0; }

#define ASSERT_EQUAL(v1, v2) if ( (v1) == (v2) ) { } else {	\
    failureFilename = (char*)__FILE__;				\
    failureLine = __LINE__;					\
    failureMessage = (char*)(#v1 " != " #v2);			\
    return 0; }

#define PASS return 1
#define FAIL(msg)	      \
  failureFilename = (char*)__FILE__;		\
  failureLine = __LINE__;     \
  failureMessage = (char*)msg;			\
  return 0;

struct testcase {
  char* name;
  int (*function)(void);
  char* failureType;
  char* file;
  int   line;
  char* message;
};

extern char* failureType;
extern char* failureFilename;
extern int   failureLine;
extern char* failureMessage;

int runSuite(char *outfilename,
             char* fixture,
             struct testcase *testcases,
             unsigned numcases);

#endif /* unittest_h */
