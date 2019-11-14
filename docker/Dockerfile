FROM python:2.7

ENV PYTHONUNBUFFERED 1

RUN apt-get update && apt-get install -y -qq \
    python-numpy \
    python-matplotlib \
    python-dev

RUN pip install numpy matplotlib

WORKDIR /workspace
