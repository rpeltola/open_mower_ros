#ifndef FILESYSTEMSERVICEINTERFACE_H
#define FILESYSTEMSERVICEINTERFACE_H

#include <FilesystemServiceInterfaceBase.hpp>

class FilesystemServiceInterface : public FilesystemServiceInterfaceBase {
 public:
  FilesystemServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx)
      : FilesystemServiceInterfaceBase(service_id, ctx) {
  }
};

#endif  // FILESYSTEMSERVICEINTERFACE_H
