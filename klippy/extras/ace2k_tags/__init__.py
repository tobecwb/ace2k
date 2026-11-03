"""One module per spool brand. Each turns the bytes of a tag into one record.

To add a brand, do three things.
1. Add a module to this package with a ``TagFormat`` subclass, and set ``FORMAT`` to that class.
2. Add one line to ``MODULES`` in ``registry.py``.
3. Add one ``[ace2k_tag <name>]`` section to ``config/ace2k_tags.cfg``.
"""
