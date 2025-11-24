# GPS API

This small Flask API stores GPS locations and serves a simple map UI.

Docker
------

This project ships with a `Dockerfile` and a `docker-compose.yml` for local development. The `Dockerfile` runs the app with `gunicorn` and the compose file mounts source for live development and persists the SQLite database file in `./data` on the host.

Build and run with `docker-compose`:

```powershell
# Build and start the service
docker compose build
docker compose up
```

This maps host port 5000 to container 5000 and mounts the repository into the container for live editing.

Persistence
-----------

The SQLite database file is created at the path configured via the `GPS_DB_PATH` environment variable. The `docker-compose.yml` sets it to `/data/gps_data.db` and mounts `./data` from your host so it persists across container rebuilds.

Direct Docker usage (without compose):

```powershell
# Build the image
docker build -t gps-api:latest .

# Run the container and map port 5000
docker run --rm -p 5000:5000 gps-api:latest
```

Development notes
-----------------

- To change the Flask launch behavior for development (automatic reload):
  - For live development with `docker-compose`, the app source is bind-mounted into the container and you can set `FLASK_ENV=development` in the `docker-compose.yml` and add `--reload` to `gunicorn` in the `Dockerfile` (or override by using a custom command in your compose file).
  - A recommended dev override in `docker-compose.yml` might be:

```yaml
    command: gunicorn -w 1 -b 0.0.0.0:5000 --reload app:app
    environment:
      - FLASK_ENV=development
```
- The map UI is served from the root route, and the JSON endpoints are:
  - `GET /gps/latest` - get the latest GPS point
  - `GET /gps/all` - get all GPS points
  - `POST /gps` - post new GPS point (JSON body)

  Installing Gunicorn (and alternatives)
  -------------------------------------

  If you'd like to run the app with Gunicorn (recommended for production), follow these steps:

  1) Create and activate a virtual environment (PowerShell):

  ```powershell
  python -m venv venv
  .\n+venv\Scripts\Activate
  python -m pip install --upgrade pip
  ```

  2) Install the project dependencies (which now include `gunicorn`):

  ```powershell
  pip install -r requirements.txt
  ```

  3) Run Gunicorn (Linux, WSL or inside Docker):

  ```powershell
  gunicorn -w 2 -b 0.0.0.0:5000 app:app
  ```

  Important: Gunicorn is not compatible with native Windows environments (it relies on POSIX features). If you're developing directly on Windows, use one of these options instead:

  - Run the app using Flask's built-in server for development (not for production):

  ```powershell
  # Using FLASK_APP environment variable
  set FLASK_APP=app
  set FLASK_ENV=development
  flask run --host=0.0.0.0
  ```

  - Use Waitress as a production-like WSGI server on Windows:

  ```powershell
  pip install waitress
  waitress-serve --port=5000 app:app
  ```

  - Or run Gunicorn inside Docker (recommended for parity with production):

  ```powershell
  docker compose build
  docker compose up
  ```

  4) After installing locally (in your venv), update `requirements.txt` with:

  ```powershell
  pip freeze > requirements.txt
  ```

  This repo pins a `gunicorn` version so Docker builds install it successfully. If you're running on Windows, prefer Docker/WSL/Waitress or the built-in Flask dev server.
