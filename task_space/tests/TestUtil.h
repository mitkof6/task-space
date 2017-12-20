#ifndef TEST_UTIL_H
#define TEST_UTIL_H

/**
 * \file This file contains some test utilities.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 *
 * @see <a href="https://simtk.org/projects/task-space">[SimTK Project]</a>, <a
 * href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */

#include <iostream>

/**
 * Test whether the supplied statement throws an std::exception of some kind,
 * and show what message got thrown.
 *
 * @param statement the statement to be executed.
 */
#define MUST_THROW(statement)                                           \
    do {                                                                \
	int threw = 0;                                                  \
	try {                                                           \
	    statement;                                                  \
	}                                                               \
	catch(const std::exception& e) {                                \
	    threw = 1;                                                  \
	    std::cout << "(OK) Threw: " << e.what() << std::endl;       \
	}                                                               \
	catch(...) {                                                    \
	    threw=2;                                                    \
	}                                                               \
	if (threw == 0) {                                               \
	    printf("The statement %s was expected to throw an exception but it did not.", #statement); \
	    throw std::exception();                                     \
	}                                                               \
	if (threw == 2) {                                               \
	    printf("The statement %s was expected to throw an std::exception but it threw something else.", #statement); \
	    throw std::exception();                                     \
	}                                                               \
    }while(false)


/**
 * Test that the supplied statement throws a particular exception.
 *
 * @param statement the statement to be executed.
 *
 * @param exc the exeption tha must be thrown
 */
#define MUST_THROW_EXCEPTION(statement, exc)                            \
    do {                                                                \
	int threw = 0;                                                  \
	try {                                                           \
	    statement;                                                  \
	}                                                               \
	catch(const exc& e) {                                           \
	    threw = 1;                                                  \
	    std::cout << "(OK) Threw: " << e.what() << std::endl;       \
	}                                                               \
	catch(...) {                                                    \
	    threw=2;                                                    \
	}                                                               \
	if (threw == 0) {                                               \
	    printf("The statement %s was expected to throw an exception but it did not.", #statement); \
	    throw std::exception();                                     \
	}                                                               \
	if (threw == 2) {                                               \
	    printf("The statement %s was expected to throw an %s exception but it threw something else.", #statement, #exc); \
	    throw std::exception();                                     \
	}                                                               \
    }while(false)

#endif
