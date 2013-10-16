/**
 * @file Utils.h
 * @author Stanislaw Maciag maciag@student.agh.edu.pl
 * @brief Miscellaneous functions
 */

#ifndef UTILS_H
#define	UTILS_H

#include <string>
#include <string.h>
#include <cstdio>
#include <cstdlib>

/**
 * Check if the input string contains number
 * @param s Input string
 * @return TRUE if input string contains valid number, FALSE otherwise
 */
bool is_number(const std::string& s);

/**
 * Converts array of bytes to double value
 * @param byteArray Input array
 * @param offset Offset from the beginning of the input array's first element to the first processed element. When equals 0, whole array,
 * from the beginning, will processed
 * @return Double value, read from array
 */
double byteArrayToDouble(char *byteArray, int offset);

/**
 * Converts double number to byte array
 * @param doubleNum Input number
 * @param byteArray Output array, must be valid pointer
 * @param offset Offset from the first element of the output array, to the element in which the number will be stored
 */
void doubleToByteArray(double doubleNum, char* byteArray, int offset);

/**
 * Converts double number to string (C style)/
 * @param doubleValue Input number
 * @param stringResult Output char array. Must be a vaild pointer
 * @return Length of output string
 */
int dtoa(double doubleValue, char* stringResult);

/**
 * Converts the char array which contains sequence of double numbers separated by delimiters to double array.
 * @param doubleArray Output array, must be a valid pointer
 * @param byteArray Input array of chars
 * @param delim Char which separates numbers in the input array
 */
void atoda(double* doubleArray, char* byteArray, const char delim);

#endif	/* UTILS_H */

