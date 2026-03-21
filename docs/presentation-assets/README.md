# Presentation assets

This folder is intentionally kept free of committed binary files so pull requests
remain text-only and compatible with review tooling that rejects binary diffs.

To generate the presentation PDFs and PNGs locally, run:

```bash
python -m pip install pillow
python scripts/export_architecture_diagrams.py
```

The script reads Mermaid diagrams from `docs/system-architecture.md` and writes:

- `deployment-networking.pdf`
- `deployment-networking.png`
- `firmware-wake-cycle.pdf`
- `firmware-wake-cycle.png`
- `server-ingest-flow.pdf`
- `server-ingest-flow.png`
- `pavewise-system-architecture-diagrams.pdf`


## Keep assets up to date on each local update

A pre-commit hook template is provided at `.githooks/pre-commit`.

To enable it in your clone, run:

```bash
git config core.hooksPath .githooks
```

After that, any commit that stages changes to the architecture markdown or export script will regenerate the local PDF/PNG assets inside `docs/presentation-assets/` before the commit completes.
