"""Build local UTF-8 reading pages; keep GitHub Markdown sources unchanged."""
from html import escape
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit, quote, unquote
import os
import re

from markdown_it import MarkdownIt

ROOT = Path(__file__).resolve().parents[1]
SHELL = '''<!doctype html>
<html lang="@LANG@"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>@TITLE@ — Quest / E6</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#fff;color:#172331;font:16px/1.75 -apple-system,BlinkMacSystemFont,"Segoe UI","Microsoft YaHei",sans-serif}
header{border-bottom:1px solid #dce2e9;padding:16px 24px;background:#f7f9fb}nav{max-width:1080px;margin:auto;display:flex;gap:22px;flex-wrap:wrap;align-items:center;font-size:14px}nav span{color:#617082;margin-right:auto}
main{max-width:1080px;margin:auto;padding:32px 28px 70px}h1,h2,h3,h4{line-height:1.35;color:#172331;scroll-margin-top:20px}h1{font-size:30px;border-bottom:1px solid #dce2e9;padding-bottom:18px;margin:0 0 24px}h2{font-size:23px;margin-top:36px}h3{font-size:19px;margin-top:26px}p,ul,ol,pre,table,blockquote{margin:15px 0}a{color:#175ca7;text-decoration:none}a:hover{text-decoration:underline}img{max-width:100%;height:auto}code{font:14px/1.6 Consolas,"Cascadia Code","Microsoft YaHei",monospace;background:#eef2f6;padding:2px 5px;border-radius:3px}pre{padding:17px 20px;overflow:auto;background:#f5f7fa;border:1px solid #e0e6ed;border-radius:5px}pre code{background:transparent;padding:0}blockquote{border-left:3px solid #9db3cc;padding:3px 18px;color:#536477;background:#f8fafc}table{border-collapse:collapse;width:100%;font-size:14px}th,td{padding:10px 12px;text-align:left;vertical-align:top;border:1px solid #dce2e9}th{background:#f3f6fa;font-weight:600}.table-wrap{overflow-x:auto}hr{border:0;border-top:1px solid #dce2e9;margin:30px 0}li+li{margin-top:5px}
@media(max-width:600px){main{padding:24px 16px 50px}header{padding:12px 16px}h1{font-size:24px}h2{font-size:21px}nav{gap:12px}}
</style></head><body><header><nav><a href="@HOME@">← Project home</a><span>@TITLE@</span><a href="@RAW@" download>Download Markdown</a></nav></header><main>@BODY@</main></body></html>'''


def build_documents(root=ROOT):
    root = Path(root).resolve()
    sources = [root / 'README.md', root / 'CHANGELOG.md', *(root / 'docs').glob('*.md'), root / 'analysis/methods.md']
    sources = [p for p in sources if p.is_file()]
    known = {p.resolve() for p in sources}
    written = []
    for source in sources:
        markdown = source.read_text(encoding='utf-8-sig')
        md = MarkdownIt('commonmark', {'html': True}).enable(['table', 'strikethrough'])

        def link_open(tokens, index, options, env):
            href = tokens[index].attrGet('href') or ''
            parts = urlsplit(href)
            if not parts.scheme and not parts.netloc and parts.path:
                target = (source.parent / unquote(parts.path)).resolve()
                if target in known:
                    href = urlunsplit(('', '', parts.path[:-3] + '.html', parts.query, parts.fragment))
                    tokens[index].attrSet('href', href)
            return md.renderer.renderToken(tokens, index, options, env)

        md.renderer.rules['link_open'] = link_open
        md.renderer.rules['table_open'] = lambda *args: '<div class="table-wrap"><table>\n'
        md.renderer.rules['table_close'] = lambda *args: '</table></div>\n'
        tokens = md.parse(markdown)
        seen = {}
        title = source.stem.replace('_', ' ').title()
        for i, token in enumerate(tokens):
            if token.type == 'heading_open' and i + 1 < len(tokens):
                label = tokens[i+1].content
                if token.tag == 'h1':
                    title = label
                slug = re.sub(r'[^\w-]+', '-', label.lower()).strip('-') or 'section'
                seen[slug] = seen.get(slug, 0) + 1
                token.attrSet('id', slug if seen[slug] == 1 else f'{slug}-{seen[slug]}')
        body = md.renderer.render(tokens, md.options, {})
        home = Path(os.path.relpath(root / 'index.html', source.parent)).as_posix()
        html = (SHELL.replace('@LANG@', 'zh-CN' if re.search(r'[\u4e00-\u9fff]', markdown) else 'en')
                .replace('@TITLE@', escape(title)).replace('@HOME@', escape(home, quote=True))
                .replace('@RAW@', quote(source.name) + '?raw=1').replace('@BODY@', body))
        destination = source.with_suffix('.html')
        destination.write_text(html, encoding='utf-8')
        written.append(destination.relative_to(root).as_posix())
    print('Rendered UTF-8 documents: ' + ', '.join(written))
    return written


if __name__ == '__main__':
    build_documents()
