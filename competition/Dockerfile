FROM python:3.11.16-slim-bookworm AS runtime
ENV PYTHONUNBUFFERED=1 MPLBACKEND=Agg OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ffmpeg libgl1 libgomp1 libxerces-c3.2 libfox-1.6-0 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /commonroad/commonroad-docker-submission
# The public scenarios and the SMP wheel are not part of the Git repository.
# Use the ones already fetched into the checkout, or download them now
# (python -m planner.fetch_data; it needs only the standard library).
COPY planner/__init__.py planner/competition_data.py planner/fetch_data.py planner/
COPY planner/assets/ planner/assets/
COPY vendor/ vendor/
RUN python -m planner.fetch_data
# Python 3.11/Linux versions from poetry.lock, including transitive dependencies.
# Keep these pins aligned when updating the lock; no requirements exports are needed.
RUN python -m pip install --no-cache-dir pip==24.0 \
    && python -m pip install --no-cache-dir \
    antlr4-python3-runtime==4.9.3 \
    asttokens==3.0.2 \
    backcall==0.2.0 \
    comm==0.2.3 \
    commonroad-clcs==2025.2.0 \
    commonroad-drivability-checker==2025.3.1 \
    commonroad-io==2024.3 \
    commonroad-route-planner==2025.1.0 \
    vendor/commonroad_search-2025.1-py3-none-any.whl \
    commonroad-sumo==2025.1.4 \
    commonroad-vehicle-models==3.0.2 \
    contourpy==1.3.3 \
    cycler==0.12.1 \
    decorator==5.3.1 \
    eclipse-sumo==1.24.0 \
    executing==2.3.0 \
    ffmpeg==1.4 \
    fonttools==4.67.0 \
    imageio==2.37.0 \
    ipython==8.10.0 \
    ipywidgets==8.1.9 \
    iso3166==3.0.0 \
    jedi==0.20.1 \
    jupyterlab-widgets==3.0.17 \
    kiwisolver==1.5.1 \
    libsumo==1.24.0 \
    lxml==6.1.3 \
    matplotlib==3.10.7 \
    matplotlib-inline==0.2.2 \
    methodtools==0.4.7 \
    networkx==3.4.2 \
    numpy==2.2.6 \
    omegaconf==2.3.0 \
    osqp==0.6.7.post3 \
    packaging==26.3 \
    parso==0.8.7 \
    pexpect==4.9.0 \
    pickleshare==0.7.5 \
    pillow==13.0.0 \
    polygon3==3.0.9.1 \
    prompt-toolkit==3.0.53 \
    protobuf==3.20.2 \
    ptyprocess==0.7.0 \
    pure-eval==0.2.4 \
    pygments==2.21.0 \
    pyparsing==3.3.3 \
    python-dateutil==3.0.0 \
    pyyaml==6.0.3 \
    qdldl==0.1.9.post1 \
    rtree==1.4.1 \
    scipy==1.15.3 \
    shapely==2.1.2 \
    six==1.17.0 \
    stack-data==0.6.3 \
    sumolib==1.24.0 \
    tqdm==4.70.1 \
    traci==1.24.0 \
    traitlets==5.16.1 \
    triangle==20250106 \
    typing-extensions==4.16.0 \
    tzdata==2026.5 \
    wcwidth==0.9.3 \
    widgetsnbextension==4.0.16 \
    wirerope==1.0.0
COPY . .
RUN python -m pip install --no-cache-dir --no-deps . && python -m pip check
ENTRYPOINT ["python", "-m", "planner"]

FROM runtime AS evaluator
RUN python -m pip install --no-cache-dir \
    iniconfig==2.3.1 \
    pluggy==1.7.0 \
    pytest==8.4.2 \
    && python -m pip check
ENTRYPOINT []
CMD ["bash", "/evaluation/build_script.sh"]

FROM evaluator AS development
# The development image adds notebooks to the evaluator's runtime and test tools.
RUN python -m pip install --no-cache-dir \
    anyio==4.15.1 \
    argon2-cffi==25.1.0 \
    argon2-cffi-bindings==26.1.0 \
    arrow==1.4.0 \
    async-lru==2.4.0 \
    attrs==26.1.0 \
    babel==2.18.0 \
    beautifulsoup4==4.15.0 \
    bleach==6.4.0 \
    certifi==2026.10.23 \
    cffi==2.1.1 \
    charset-normalizer==3.5.2 \
    debugpy==1.8.22 \
    defusedxml==0.7.1 \
    fastjsonschema==2.22.2 \
    fqdn==1.6.0 \
    h11==0.16.0 \
    httpcore==1.0.9 \
    httpx==0.28.1 \
    idna==3.20 \
    ipykernel==6.29.5 \
    isoduration==20.11.0 \
    jinja2==3.1.6 \
    json5==0.17.3 \
    jsonpointer==3.4 \
    jsonschema==4.26.0 \
    jsonschema-specifications==2025.9.1 \
    jupyter-builder==1.2.3 \
    jupyter-client==8.10.0 \
    jupyter-core==5.9.1 \
    jupyter-events==0.12.1 \
    jupyter-lsp==2.3.1 \
    jupyter-server==2.21.1 \
    jupyter-server-terminals==0.5.4 \
    jupyterlab==4.6.4 \
    jupyterlab-pygments==0.3.0 \
    jupyterlab-server==2.28.1 \
    lark==1.3.1 \
    markupsafe==3.0.4 \
    mistune==3.4.0 \
    nbclient==0.11.0 \
    nbconvert==7.17.2 \
    nbformat==5.11.1 \
    nest-asyncio==1.6.0 \
    notebook-shim==0.2.4 \
    overrides==7.7.0 \
    pandocfilters==1.5.1 \
    platformdirs==4.13.5 \
    prometheus-client==0.26.0 \
    psutil==7.2.2 \
    pycparser==3.12 \
    python-json-logger==4.2.0 \
    pyzmq==27.2.0 \
    referencing==0.37.0 \
    requests==2.34.2 \
    rfc3339-validator==0.1.4 \
    rfc3986-validator==0.1.1 \
    rfc3987-syntax==1.1.0 \
    rpds-py==2026.9.1 \
    send2trash==2.1.0 \
    soupsieve==2.10 \
    terminado==0.18.1 \
    tinycss2==1.5.1 \
    tornado==6.6 \
    uri-template==1.3.0 \
    urllib3==2.8.0 \
    webcolors==25.10.0 \
    webencodings==0.6.1 \
    websocket-client==1.9.2 \
    && python -m pip install --no-deps -e . && python -m pip check
CMD ["sleep", "infinity"]

FROM runtime AS submission
