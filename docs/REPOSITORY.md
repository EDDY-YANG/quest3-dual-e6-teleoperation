# Repository guide

This repository contains the Quest 3 / dual DOBOT E6 project website, five development videos, editable figures, documentation and historical-log statistics. It is a static publication: it has no connection to a robot and sends no motion commands.

[Download the Windows x64 SDK](https://github.com/EDDY-YANG/quest3-dual-e6-teleoperation/releases/latest/download/quest-e6-sdk-windows-x64.zip) for the editable control source, VS Code workspace and prebuilt native solver. [Install and check the SDK](SDK_INSTALL.md), then follow [examples 01–06](SDK_EXAMPLES.md). The website sources and the downloadable SDK are distributed separately.

## View locally

Open the published project website in a browser, or preview this checkout with Python 3.10 or newer:

```bash
python tools/serve_preview.py
```

Then open `http://127.0.0.1:8266/`. On Windows, `START_PREVIEW.cmd` launches the same preview in the background. This previews the static website only.

## Edit and rebuild

Viewing the supplied HTML requires no Python packages. To regenerate the figures, README and formatted documentation, use Python 3.10 or newer in a virtual environment:

```bash
python -m venv .venv
```

Activate `.venv` using `.venv\Scripts\Activate.ps1` in PowerShell, or `source .venv/bin/activate` on macOS/Linux. Then run:

```bash
python -m pip install -r requirements.txt
python tools/draw_figures.py
python tools/build_page.py
```

- `page.template.html` controls the project-page layout and prose.
- `tools/draw_figures.py` generates the two architecture figures, reusing the supplied E6 image. See [figure sources](FIGURE_SOURCES.md) for the optional mesh input.
- `tools/build_page.py` generates the performance figure, comparison table, README and document HTML pages from the retained summaries.
- Edit documentation in its `.md` source; `python tools/render_docs.py` regenerates its `.html` reading page.
- `analysis/results_v4/` contains the published summaries, per-record metrics, source hashes and scalar metric distributions. The historical raw logs are not redistributed.

Recomputing all statistics requires the original recordings; the script accepts their directory explicitly. The [statistical methods](../analysis/methods.md) document eligibility, exclusions and limitations. The published summaries alone support rebuilding the website and figures, not replaying the physical experiments.

The build and local preview were checked on Windows. macOS/Linux instructions use standard Python tooling but have not been executed on those platforms.

## Publication

GitHub Pages serves the generated files from the `main` branch at the repository root. `.nojekyll` retains the static asset layout. Rebuild before committing edited Markdown or figure sources.

Authoring code and documentation use the root MIT license. The E6 rendering retains the upstream notice in `assets/E6_MODEL_LICENSE`; see [figure sources](FIGURE_SOURCES.md) and [media provenance](media_manifest.json).
