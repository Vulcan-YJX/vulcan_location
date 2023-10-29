#include "vpi/Status.h"

#ifndef VPI_UTILS_HPP__
#define VPI_UTILS_HPP__

#define CHECK_STATUS(STMT)                              \
  do {                                                  \
    VPIStatus status = (STMT);                          \
    if (status != VPI_SUCCESS) {                        \
      char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];       \
      vpiGetLastStatusMessage(buffer, sizeof(buffer));  \
      std::ostringstream ss;                            \
      ss << vpiStatusGetName(status) << ": " << buffer; \
      throw std::runtime_error(ss.str());               \
    }                                                   \
  } while (0);

#endif /*VPI_UTILS_HPP__*/
