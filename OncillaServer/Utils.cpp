#include "Utils.h"

bool is_number(const std::string& s) {
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it))
        ++it;
    return !s.empty() && it == s.end();
}

double byteArrayToDouble(char *byteArray, int offset) {

    double result;
    memcpy(&result, byteArray + offset, sizeof (double));
    return result;

}

void doubleToByteDouble(double doubleNum, char* byteArray, int offset) {

    memcpy(byteArray + offset, &doubleNum, sizeof (doubleNum));

}

int dtoa(double doubleValue, char* stringResult) {
    return sprintf(stringResult, "%f", doubleValue);
}

void atoda(double *doubleArray, char* byteArray, const char delim) {

    int length = 0, i = 0;

    while (byteArray[i] != '\0') {
        if (byteArray[i] == delim)
            length++;
        i++;
    }

    length++;

    i = 0;

    char buffer[256];
    bzero(buffer, 256);
    int j = 0, n = 0;

    while (byteArray[i] != '\0') {

        if (byteArray[i] != delim) {

            buffer[j] = byteArray[i];
            j++;

        } else {

            doubleArray[n] = atof(buffer);
            bzero(buffer, 256);
            j = 0;
            n++;

        }

        i++;
    }

    doubleArray[n] = atof(buffer);

}