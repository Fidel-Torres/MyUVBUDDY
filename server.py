import sys
import logging
import argparse
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from functools import partial

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)


class LoggingHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        logger.info("%s - %s", self.address_string(), format % args)

    def log_error(self, format, *args):
        logger.error("%s - %s", self.address_string(), format % args)


def parse_args():
    parser = argparse.ArgumentParser(description="Simple HTTP file server")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--directory", default=".")
    return parser.parse_args()


def main():
    args = parse_args()

    handler = partial(LoggingHandler, directory=args.directory)

    try:
        server = ThreadingHTTPServer((args.host, args.port), handler)
    except OSError as e:
        logger.error("Could not start server: %s", e)
        sys.exit(1)

    logger.info("Serving '%s' at http://%s:%d", args.directory, args.host, args.port)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        server.shutdown()


if __name__ == "__main__":
    main()