FROM squidfunk/mkdocs-material

ARG REQUIREMENTS=requirements.txt

COPY ${REQUIREMENTS} requirements.txt
RUN pip install --no-cache-dir -r requirements.txt
