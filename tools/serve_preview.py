"""Serve only this publication directory on loopback. No robot SDK imports."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, parse_qs, quote


class PreviewHandler(SimpleHTTPRequestHandler):
    """Serve explicit UTF-8 and preserve old Markdown URLs as reading links."""

    def guess_type(self, path):
        suffix = Path(path).suffix.lower()
        types = {'.html': 'text/html', '.htm': 'text/html', '.md': 'text/markdown',
                 '.txt': 'text/plain', '.py': 'text/plain', '.ps1': 'text/plain',
                 '.csv': 'text/csv', '.json': 'application/json', '.css': 'text/css',
                 '.js': 'application/javascript', '.svg': 'image/svg+xml'}
        if suffix in types:
            return types[suffix] + '; charset=utf-8'
        return super().guess_type(path)

    def send_head(self):
        parts = urlsplit(self.path)
        source = Path(self.translate_path(parts.path))
        raw = parse_qs(parts.query).get('raw') == ['1']
        if source.suffix.lower() == '.md' and source.is_file() and not raw:
            if source.with_suffix('.html').is_file():
                self.send_response(302)
                self.send_header('Location', parts.path[:-3] + '.html')
                self.send_header('Content-Length', '0')
                self.end_headers()
                return None
        return super().send_head()

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        parts = urlsplit(self.path)
        if parse_qs(parts.query).get('raw') == ['1']:
            name = Path(self.translate_path(parts.path)).name
            self.send_header('Content-Disposition', "attachment; filename*=UTF-8''" + quote(name))
        super().end_headers()

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port',type=int,default=8266)
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
server=ThreadingHTTPServer(('127.0.0.1',args.port),partial(PreviewHandler,directory=str(root)))
print(f'Local preview: http://127.0.0.1:{server.server_port}/',flush=True)
print('No upload. Press Ctrl+C to stop this preview.',flush=True)
try:server.serve_forever()
except KeyboardInterrupt:pass
finally:server.server_close()
