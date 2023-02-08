# For interacting with jupter notebooks. This installs a kernel for python and guile.

# Build from opencog-dev:cli as it has all packages it needs.
FROM opencog/opencog-dev:cli

RUN apt-get update --fix-missing && apt-get install -y bzip2 \
    libglib2.0-0 libxext6 libsm6 libxrender1 \
    mercurial subversion python3-pip

# To install jupyter notebook alongside with kernel on default python3 for later usage
# torando must be at this specific version from https://github.com/jupyter/notebook/issues/3407
RUN python3 -m pip install jupyter ipykernel tornado==4.5.3

# Install guile-json as it is needed for guile-kernel
WORKDIR /opt/

RUN wget https://download.savannah.gnu.org/releases/guile-json/guile-json-0.6.0.tar.gz && \
    tar xvf guile-json-0.6.0.tar.gz && \
    rm guile-json-0.6.0.tar.gz && \
    cd guile-json-0.6.0 && \
    ./configure && \
    make install

# ZMQ library to enable communication with jupyter notebook
RUN wget https://raw.githubusercontent.com/jerry40/guile-simple-zmq/master/src/simple-zmq.scm -O /usr/local/share/guile/site/simple-zmq.scm

# Create a directory with the guile kernel in jupyter notebook kernels directory
WORKDIR /usr/local/share/jupyter/kernels

RUN git clone --depth 1 https://github.com/jerry40/guile-kernel
# Copy kernel description to the above folder
COPY kernel.json /usr/local/share/jupyter/kernels/guile-kernel

EXPOSE 8888
USER opencog
CMD [ "jupyter", "notebook", "--ip=0.0.0.0", "--port=8888" ]
