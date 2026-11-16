# Third-party notice

`chess_brain_handover/_vendor/chess/` is a vendored copy of **python-chess 1.11.2** (https://github.com/niklasf/python-chess,
author Niklas Fiekas), provisioned by `setup.py` at build time. python-chess is licensed **GPL-3.0-or-later**; this package's
own code is labelled MIT (see `LICENSE`). Because GPL code is bundled with it, check with your supervisor/team how the
combined work should be licensed before publishing it publicly, and ship python-chess's licence text with any redistribution.
Not legal advice. The vendored directory is excluded from Git by `.gitignore`; the first build downloads it (internet needed) unless it is already present.
