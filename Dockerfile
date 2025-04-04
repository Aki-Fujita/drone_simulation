FROM python:3
USER root

RUN apt-get update
RUN apt-get -y install locales && \
  libgl1-mesa-glx \
  libglib2.0-0 \
  localedef -f UTF-8 -i ja_JP ja_JP.UTF-8 
ENV LANG ja_JP.UTF-8
ENV LANGUAGE ja_JP:ja
ENV LC_ALL ja_JP.UTF-8
ENV TZ JST-9
ENV TERM xterm

RUN apt-get install -y vim less
COPY requirements.txt .
RUN pip install --upgrade pip
RUN pip install --upgrade setuptools
RUN pip install -r requirements.txt
RUN pip install --upgrade ipywidgets tqdm

RUN rm requirements.txt
RUN python -m pip install jupyterlab