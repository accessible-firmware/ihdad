# AI/LLM Policy

The initial work was done entirely by hand and before LLMs gained a lot of traction.
Th^e port of the driver from RedoxOS to a freestanding binary was also done 100% by hand.

Claude Opus 4.7/4.8 was used to find hardware quirks and implement the fixes from FreeBSD/OpenBSD/Linux while preseving the previous RedoxOS inspired achitecture _and not copying code from Linux_; only high-level concepts, not code were borrowed, preseving the non-GPL nature of this crate which is essential.
Some comments written about the quirks are also generated from an LLM, fed primarily by the detailed trace logs while running this library on real hardware.

## Your Work

You _must_ state whether AI/LLMs were used for the code generation, documentation, or PR description along with which model(s) were used in its creation.
You agree, implicitly, that _you **and only you**_ are responsible for said code and that you will be the primary contact if your code causes any issues down the line.
When asked for why in your words, "Claude did it, idk" (as well as AI-generated responses to these questions) are _never_ acceptable and will result in a permenant ban from the organization and all associated repositories.
