Import("env")

import os
import subprocess


def get_git_info(project_dir):
    def git(*args):
        return subprocess.check_output(
            ["git"] + list(args), cwd=project_dir, text=True, stderr=subprocess.DEVNULL
        ).strip()

    try:
        commit_hash = git("rev-parse", "HEAD")
        author = git("log", "-1", "--format=%an")
        uploader = git("config", "user.name")
        return commit_hash, author, uploader
    except subprocess.CalledProcessError:
        return "unknown", "unknown", "unknown"


project_dir = env["PROJECT_DIR"]
commit_hash, author, uploader = get_git_info(project_dir)

# First 8 bytes of the hash packed into a uint64_t — fits in one CAN frame.
hash_u64 = f"0x{commit_hash[:16]}ULL" if commit_hash != "unknown" else "0ULL"

header_dir = os.path.join(project_dir, "include", "generated")
os.makedirs(header_dir, exist_ok=True)
header_path = os.path.join(header_dir, "git_info.h")

with open(header_path, "w") as f:
    f.write(f"""\
#pragma once
#include <stdint.h>

#define GIT_COMMIT_HASH   "{commit_hash}"
#define GIT_COMMIT_HASH_U64 {hash_u64}
#define GIT_COMMIT_AUTHOR "{author}"
#define GIT_UPLOADER      "{uploader}"
""")

print(f"[git_info] hash={commit_hash[:8]}  author={author}  uploader={uploader}")
