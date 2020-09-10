

Sensirion driver from https://github.com/Sensirion/embedded-sgp.
As including external repos is rather comlicated/messy with Mynewt, files are
copied insted of git submodule.  files from git tag "6.0.0"
(2dd80bcb0eec28273a531510672ba65436343b9d) copied and renamed as follows:

```
embedded-sgp/sgp30/sgp30.h -> src/sensirion_sgp30.h (also changed include directives)
embedded-sgp/sgp30/sgp30.c -> src/sensirion_sgp30.c
embedded-sgp/sgp-common/sgp_git_version.h -> src/sgp_git_version.h
embedded-sgp/embedded-common/sensirion_common.h -> src/sensirion_common.h
embedded-sgp/embedded-common/sensirion_i2c.h -> src/sensirion_i2c.h
embedded-sgp/embedded-common/sensirion_arch_config.h -> src/sensirion_arch_config.h

```

