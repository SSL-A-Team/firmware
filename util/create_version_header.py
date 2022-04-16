#!/usr/bin/env python3

import re

from git import Repo

# 1st captuing group X.X.X (MAJOR.MINOR.PATCH)
#   2nd capturing group MAJOR version
#   3rd capturing group MINOR version
#   4th capturing group PATCH
# 5th capturing group -N-H (-<number of commits since version tag>-<latest object hash>)
#   6th capturing group N, number of commits
#   7th capturing group H, the hash reference
# 8th capturing group -dirty
TAGGED_REGEX="^v((\d+)\.(\d+)\.(\d+))(-(\d+)-g([0-9a-f]{7,40}))(-dirty)?$"
STDINT_UINT8_CAPACITY = ((2 ** 8) - 1)
STDINT_UINT16_CAPACITY = ((2 ** 16) - 1)



def generate_header():
	gen_string = ""
	gen_string += "#pragma once"


def main():
	fw_repo = Repo()
	assert not fw_repo.bare

	print(f"Found repo: {fw_repo}")
	has_untracked = len(fw_repo.untracked_files) != 0
	is_dirty = fw_repo.is_dirty()
	is_clean = not (is_dirty or has_untracked)

	most_recent_tag_info = fw_repo.git.describe('--dirty')
	print(f"Recent Tag Info: {most_recent_tag_info}")

	tag_regex = re.compile(TAGGED_REGEX)
	tag_match_result = tag_regex.match(most_recent_tag_info)
	if (not tag_match_result):
		print("invalid tag information")
		exit(1)

	print(tag_match_result.groups())
	full_version = tag_match_result[0]
	major_version = int(tag_match_result[1])
	minor_version = int(tag_match_result[2])
	patch_version = int(tag_match_result[3])
	additional_commits = int(tag_match_result[5])
	latest_commit_hash = tag_match_result[6]

	if (major_version > STDINT_UINT8_CAPACITY):
		print(f"Major Version ({major_version}) is too damn high!")
		exit(1)

	if (minor_version > STDINT_UINT8_CAPACITY):
		print(f"Minor Version ({minor_version}) is too damn high!")
		exit(1)

	if (patch_version > STDINT_UINT16_CAPACITY):
		print(f"Patch Version ({patch_verison}) is too damn high!")
		exit(1)

	if (len(latest_commit_hash) > 8):
		latest_commit_hash_full = latest_commit_hash
		latest_commit_hash = latest_commit_hash[0:8]
		print(f"WARNING: commit hash is being truncated to 8 characters. It may not be unique ({latest_commit_hash_full} -> {latest_commmit_hash}).")	
	latest_commit_hash_int = int(latest_commit_hash, 16)


if __name__ == "__main__":
	main()
