# Uses gcc8 for glibc 2.28
ARG FROM=conanio/gcc10:latest
FROM ${FROM} AS lanelet2_conan_deps

ENV DEBIAN_FRONTEND noninteractive
ENV CMAKE_POLICY_VERSION_MINIMUM 3.5

# install requirements for python installation
RUN sudo -E apt-get update \
  && sudo -E apt-get install -y libssl-dev zlib1g-dev \
  libbz2-dev libreadline-dev libsqlite3-dev curl \
  libncursesw5-dev xz-utils libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev

# setup python environment
ARG PY_VERSION=3.12
RUN pyenv update \
  && pyenv install -s $PY_VERSION \
  && pyenv global $PY_VERSION
# numpy build-time ABI policy, mirroring the boost version picked in conanfile.py:
# - python >= 3.9 builds against boost 1.87 and numpy 2. Building against numpy 2 headers keeps the
#   extension usable with runtime numpy >= 1.19 as well, so one wheel serves both numpy generations.
#   This is also what makes python 3.13 possible at all, since numpy 1 has no cp313 wheels. On 3.9 this
#   resolves to the numpy 2.0.x series, the last one supporting it.
# - python 3.8 builds against boost 1.75 and stays on numpy 1, since numpy 2 requires python >= 3.9.
#   oldest-supported-numpy picks the oldest ABI there, which is forward compatible within numpy 1.x.
# conan 2.9.2 is the first release whose default conancenter remote points at https://center2.conan.io.
# The old https://center.conan.io was frozen and carries no recipe newer than boost 1.86.0, so an older
# client cannot resolve the boost version needed for the numpy 2 build.
# urllib3 v2 refuses to import unless the ssl module was built against OpenSSL 1.1.1+, and the images
# used for python 3.8 and 3.9 (conanio/gcc7, see the matrix in .github/workflows/cd.yaml) only have
# OpenSSL 1.0.2g. conan accepts urllib3 >=1.26.6,<3, so keep it on 1.x there.
RUN pip install "conan~=2.31.0" catkin_pkg wheel auditwheel cmake \
  "urllib3<2; python_version < '3.10'" \
  "oldest-supported-numpy; python_version < '3.9'" \
  "numpy>=2.0; python_version >= '3.9'"

# install patchelf
RUN wget https://github.com/NixOS/patchelf/releases/download/0.17.2/patchelf-0.17.2-x86_64.tar.gz \
  && tar -xzf patchelf-0.17.2-x86_64.tar.gz \
  && sudo -E ln -s $HOME/bin/patchelf /bin/patchelf \
  && patchelf --version


FROM lanelet2_conan_deps as lanelet2_conan_src

# checkout code
RUN mkdir src
WORKDIR /home/conan/src
COPY --chown=1000:1001 . lanelet2/

FROM lanelet2_conan_src as lanelet2_conan

# compile
ARG CONAN_ARGS=""
ARG PLATFORM="manylinux_2_31_x86_64"
WORKDIR /home/conan/src/lanelet2
# b2 is an application package, so conancenter ships one prebuilt binary for it instead of building it
# per profile. That binary needs GLIBC_2.34, which none of the build images provide (gcc7 has 2.23,
# gcc10 has 2.31), so build this one tool from source everywhere.
RUN conan profile detect \
  && conan create . --format=json --build=missing --build="b2/*" -o "&:build_wheel=True" -o "&:platform=${PLATFORM}" ${CONAN_ARGS} > conaninfo.json

# obtain wheel
FROM lanelet2_conan as lanelet2_conan_with_pip_wheel

SHELL ["/bin/bash", "-c"]
WORKDIR /home/conan
RUN LANELET2_PACKAGE_DIR=$(python3 -c "import json; f=open('src/lanelet2/conaninfo.json'); data=json.load(f); ll2=next(d for d in data['graph']['nodes'].values() if 'lanelet2' in d['ref']); print(f'{ll2[\"package_folder\"]}/wheel')") \
  && cp -r $LANELET2_PACKAGE_DIR dist

# to extract the wheel manually run:
# $ docker run --rm -v /path/to/some/local/folder:/dist <image> /bin/bash -c 'sudo cp dist/lanelet2-<...>.whl /dist/.'