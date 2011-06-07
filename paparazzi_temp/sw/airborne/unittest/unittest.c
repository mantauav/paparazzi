/** \file
    Unit Test Framework Routines
    $Id$
    ***Warning*** Aurora Flight Sciences, Inc. Proprietary Material
    Author: Jim Francis
    Copyright (C) 2010 by Aurora Flight Sciences, Inc. All Rights Reserved.
*/

#include <stdio.h>
#include "unittest.h"

char* failureFilename = 0;
int   failureLine;
char* failureMessage = 0;

int runSuite(char *outfilename,
	     char *fixture,
	     struct testcase *testcases,
	     unsigned numcases)
{
  int ii;
  int failuresTotal = 0;
  int errors = 0;
  int failures = 0;
  FILE* outfile;
  
  for (ii = 0;
       ii < numcases;
       ii++) {
    
    testcases[ii].failureType = 0;
    if ((*testcases[ii].function)() != 1) {
      testcases[ii].failureType = (char*) "Assertion";
      testcases[ii].file = failureFilename;
      testcases[ii].line = failureLine;
      testcases[ii].message = failureMessage;
    }
  }
  
  if (outfilename != 0) {
    outfile = fopen(outfilename,"w");
  } else {
    outfile = stdout;
  }
  
  fprintf(outfile,"<?xml version=\"1.0\" encoding='ISO-8859-1' standalone='yes' ?>\n");
  fprintf(outfile,"<TestRun>\n");
  fprintf(outfile,"  <FailedTests>\n");
  for (ii = 0;
       ii < numcases;
       ii++) {
    
    if (testcases[ii].failureType != 0) {
      failures++;
      failuresTotal++;
      errors++;
      
      fprintf(outfile,"    <FailedTest id=\"%d\">\n",ii);
      fprintf(outfile,"      <Name>%s::%s</Name>\n",fixture,testcases[ii].name);
      fprintf(outfile,"      <FailureType>%s</FailureType>\n",
	      testcases[ii].failureType);
      fprintf(outfile,"      <Location>\n");
      fprintf(outfile,"        <File>%s</File>\n",testcases[ii].file);
      fprintf(outfile,"        <Line>%d</Line>\n",testcases[ii].line);
      fprintf(outfile,"      </Location>\n");
      fprintf(outfile,"      <Message>%s</Message>\n",
	      testcases[ii].message);
      fprintf(outfile,"    </FailedTest>\n");
      
      /* also write to stderr to make it easier to find problems
	 without having to drop into the XML file
      */
      fprintf(stderr,"%s: In function '%s':\n",
	      testcases[ii].file,
	      testcases[ii].name);
      fprintf(stderr,"%s:%d: error: %s\n",
	      testcases[ii].file, testcases[ii].line,
	      testcases[ii].message);
    }
  }
  fprintf(outfile,"  </FailedTests>\n");
  
  fprintf(outfile,"  <SuccessfulTests>\n");
  for (ii = 0;
       ii < numcases;
       ii++) {
    if (testcases[ii].failureType == 0) {
      fprintf(outfile,"    <Test id=\"%d\">\n",ii);
      fprintf(outfile,"      <Name>%s::%s</Name>\n",fixture,testcases[ii].name);
      fprintf(outfile,"    </Test>\n");
    }
  }
  fprintf(outfile,"  </SuccessfulTests>\n");
  
  fprintf(outfile,"  <Statistics>\n");
  fprintf(outfile,"    <Tests>%u</Tests>\n",numcases);
  fprintf(outfile,"    <FailuresTotal>%d</FailuresTotal>\n",failuresTotal);
  fprintf(outfile,"    <Errors>%d</Errors>\n",errors);
  fprintf(outfile,"    <Failures>%d</Failures>\n",failures);
  fprintf(outfile,"  </Statistics>\n");
  
  fprintf(outfile,"</TestRun>\n");
  
  fclose(outfile);
  
  return failuresTotal;
}
