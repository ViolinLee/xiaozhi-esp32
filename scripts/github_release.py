#!/usr/bin/env python3
"""
GitHub Release发布脚本
将releases目录下的zip文件发布到GitHub Release
"""

import os
import sys
import json
import argparse
import subprocess
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

# 从 scripts/.env 加载尚未设置的环境变量，不依赖第三方包。
script_dir = os.path.dirname(os.path.abspath(__file__))
env_path = os.path.join(script_dir, '.env')
if os.path.exists(env_path):
    with open(env_path, 'r', encoding='utf-8') as env_file:
        for line in env_file:
            line = line.strip()
            if line and not line.startswith('#') and '=' in line:
                key, value = line.split('=', 1)
                os.environ.setdefault(key.strip(), value.strip())
    print(f"✓ 已从 {env_path} 加载环境变量")

# 切换到项目根目录
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

GITHUB_API_BASE = "https://api.github.com"
RELEASES_DIR = "releases"


def get_github_token():
    """从环境变量获取GitHub token"""
    token = os.environ.get('GITHUB_TOKEN')
    if not token:
        print("错误: 未设置GITHUB_TOKEN环境变量")
        print("请选择以下方式之一设置:")
        print("  1. 在 scripts/.env 文件中添加: GITHUB_TOKEN=your_token")
        print("  2. 设置环境变量: export GITHUB_TOKEN=your_token")
        sys.exit(1)
    return token


def get_repo_info():
    """从git配置获取仓库信息"""
    try:
        # 获取remote url
        result = subprocess.run(
            ['git', 'config', '--get', 'remote.origin.url'],
            capture_output=True,
            text=True,
            check=True
        )
        url = result.stdout.strip()

        # 解析owner和repo
        # 支持 https://github.com/owner/repo.git 或 git@github.com:owner/repo.git
        if url.startswith('https://'):
            parts = url.replace('https://github.com/', '').replace('.git', '').split('/')
        elif url.startswith('git@'):
            parts = url.replace('git@github.com:', '').replace('.git', '').split('/')
        else:
            raise ValueError(f"无法解析仓库URL: {url}")

        if len(parts) != 2:
            raise ValueError(f"无法解析仓库信息: {url}")

        return parts[0], parts[1]
    except Exception as e:
        print(f"错误: 无法获取仓库信息: {e}")
        print("请确保在git仓库中运行此脚本")
        sys.exit(1)


def github_request(method, url, token, payload=None, data=None, content_type=None):
    """调用 GitHub API，并将错误响应转换为可读异常。"""
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        content_type = "application/json"

    headers = {
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json",
        "X-GitHub-Api-Version": "2022-11-28",
    }
    if content_type:
        headers["Content-Type"] = content_type

    request = Request(url, data=data, headers=headers, method=method)
    try:
        with urlopen(request) as response:
            response_data = response.read()
            return json.loads(response_data) if response_data else None
    except HTTPError as error:
        response_data = error.read().decode("utf-8", errors="replace")
        try:
            message = json.loads(response_data).get("message", response_data)
        except json.JSONDecodeError:
            message = response_data or str(error)
        raise RuntimeError(f"GitHub API {error.code}: {message}") from error
    except URLError as error:
        raise RuntimeError(f"GitHub API连接失败: {error.reason}") from error


def get_release(owner, repo, tag, token):
    """按标签获取 release；不存在时返回 None。"""
    url = f"{GITHUB_API_BASE}/repos/{owner}/{repo}/releases/tags/{tag}"
    try:
        return github_request("GET", url, token)
    except RuntimeError as error:
        if "GitHub API 404:" in str(error):
            return None
        raise


def create_release(
    owner,
    repo,
    tag,
    token,
    target_commitish,
    draft=False,
    prerelease=False,
    body=None,
):
    """创建GitHub release"""
    url = f"{GITHUB_API_BASE}/repos/{owner}/{repo}/releases"
    data = {
        'tag_name': tag,
        'name': tag,
        'target_commitish': target_commitish,
        'draft': draft,
        'prerelease': prerelease
    }
    if body:
        data['body'] = body
    return github_request("POST", url, token, payload=data)


def upload_asset(upload_url, file_path, token):
    """上传文件到GitHub release"""
    upload_base = upload_url.split("{", 1)[0]
    asset_name = quote(os.path.basename(file_path))
    with open(file_path, "rb") as file:
        return github_request(
            "POST",
            f"{upload_base}?name={asset_name}",
            token,
            data=file.read(),
            content_type="application/octet-stream",
        )


def update_release(release, token, draft, prerelease, body):
    """更新已有 release 的发布属性和说明。"""
    payload = {
        "name": release["tag_name"],
        "draft": draft,
        "prerelease": prerelease,
        "body": body or "",
    }
    return github_request("PATCH", release["url"], token, payload=payload)


def delete_matching_assets(release, file_paths, token):
    """删除已有 release 中与本次上传同名的资产。"""
    names = {os.path.basename(path) for path in file_paths}
    for asset in release.get("assets", []):
        if asset["name"] in names:
            print(f"删除已有资产: {asset['name']}")
            github_request("DELETE", asset["url"], token)


def get_release_info(zip_path):
    """从zip文件名获取release信息"""
    filename = os.path.basename(zip_path)
    # 格式: v2.0.2_nodehexa.zip
    if not filename.startswith('v') or not filename.endswith('.zip'):
        return None, None

    name_without_ext = filename[:-4]  # 去掉.zip
    parts = name_without_ext.split('_', 1)

    if len(parts) == 2:
        tag = parts[0]  # v2.0.2
        board = parts[1]  # nodehexa
    else:
        tag = name_without_ext
        board = None

    return tag, board


def generate_release_notes(tag, previous_tag=None):
    """从git log生成release notes"""
    try:
        if previous_tag:
            # 获取两个tag之间的提交
            cmd = ['git', 'log', f'{previous_tag}..HEAD', '--pretty=format:* %s (%h)']
        else:
            # 获取最近的提交
            cmd = ['git', 'log', '--max-count=20', '--pretty=format:* %s (%h)']

        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        commits = result.stdout.strip()

        if commits:
            return f"## 更新内容\n\n{commits}"
        else:
            return None
    except Exception as e:
        print(f"生成release notes时出错: {e}")
        return None


def read_body_from_file(file_path):
    """从文件读取release notes"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.read().strip()
    except Exception as e:
        print(f"读取文件失败: {e}")
        return None


def publish_release(
    file_paths,
    owner,
    repo,
    tag,
    token,
    target_commitish,
    draft=False,
    prerelease=False,
    overwrite=False,
    body=None,
):
    """创建一个 release，并向其中上传同版本的全部固件。"""
    print(f"\n处理 Release: {tag}")
    release = get_release(owner, repo, tag, token)
    if release and not overwrite:
        print(f"Release {tag} 已存在，跳过 (使用 --overwrite 强制覆盖)")
        return False

    if release:
        print(f"更新 Release: {tag}")
        delete_matching_assets(release, file_paths, token)
        release = update_release(release, token, draft, prerelease, body)
    else:
        print(f"创建 Release: {tag} -> {target_commitish}")
        release = create_release(
            owner,
            repo,
            tag,
            token,
            target_commitish,
            draft=draft,
            prerelease=prerelease,
            body=body,
        )

    for file_path in file_paths:
        print(f"上传文件: {os.path.basename(file_path)}")
        upload_asset(release["upload_url"], file_path, token)
    print(f"✓ 成功发布 {tag}，共 {len(file_paths)} 个固件")
    return True


def list_releases(owner, repo, token):
    """列出所有可用的zip文件"""
    if not os.path.exists(RELEASES_DIR):
        print(f"目录 {RELEASES_DIR} 不存在")
        return

    zip_files = [f for f in os.listdir(RELEASES_DIR) if f.endswith('.zip') and f.startswith('v')]

    if not zip_files:
        print(f"在 {RELEASES_DIR} 目录下未找到zip文件")
        return

    print(f"\n找到 {len(zip_files)} 个zip文件:")
    for i, filename in enumerate(sorted(zip_files), 1):
        tag, board = get_release_info(os.path.join(RELEASES_DIR, filename))
        board_info = f" ({board})" if board else ""
        print(f"  {i}. {filename} -> {tag}{board_info}")


def main():
    parser = argparse.ArgumentParser(
        description='将releases目录下的zip文件发布到GitHub Release',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 发布所有zip文件
  python scripts/github_release.py --all

  # 发布特定文件
  python scripts/github_release.py releases/v2.0.2_nodehexa.zip

  # 作为草稿发布
  python scripts/github_release.py --all --draft

  # 作为预发布版本
  python scripts/github_release.py --all --prerelease

  # 覆盖已存在的release
  python scripts/github_release.py --all --overwrite

  # 添加release notes (直接指定)
  python scripts/github_release.py --all --body "修复了重要bug"

  # 从文件读取release notes
  python scripts/github_release.py --all --body-file CHANGELOG.md

  # 自动从git log生成release notes
  python scripts/github_release.py --all --auto-notes

  # 从指定tag到当前生成release notes
  python scripts/github_release.py --all --auto-notes --previous-tag v2.0.1

  # 列出所有可用的zip文件
  python scripts/github_release.py --list

环境变量:
  GITHUB_TOKEN: GitHub personal access token (必需)
        """
    )

    parser.add_argument(
        'files',
        nargs='*',
        help='要发布的zip文件路径 (如果不指定，需要配合--all使用)'
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='发布releases目录下的所有zip文件'
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='列出所有可用的zip文件'
    )
    parser.add_argument(
        '--draft',
        action='store_true',
        help='作为草稿发布'
    )
    parser.add_argument(
        '--prerelease',
        action='store_true',
        help='作为预发布版本发布'
    )
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='覆盖已存在的release'
    )
    parser.add_argument(
        '--owner',
        help='GitHub仓库所有者 (默认从git配置自动获取)'
    )
    parser.add_argument(
        '--repo',
        help='GitHub仓库名称 (默认从git配置自动获取)'
    )
    parser.add_argument(
        '--body',
        help='Release notes内容 (直接指定)'
    )
    parser.add_argument(
        '--body-file',
        help='从文件读取release notes'
    )
    parser.add_argument(
        '--auto-notes',
        action='store_true',
        help='自动从git log生成release notes'
    )
    parser.add_argument(
        '--previous-tag',
        help='用于生成release notes的上一个tag (配合--auto-notes使用)'
    )
    parser.add_argument(
        '--target',
        help='Release标签指向的提交（默认当前HEAD）'
    )

    args = parser.parse_args()

    # 列出文件
    if args.list:
        list_releases(None, None, None)
        return

    # 获取GitHub token
    token = get_github_token()

    # 获取仓库信息
    if args.owner and args.repo:
        owner, repo = args.owner, args.repo
    else:
        owner, repo = get_repo_info()

    print(f"仓库: {owner}/{repo}")

    # 确定要发布的文件列表
    if args.all:
        if not os.path.exists(RELEASES_DIR):
            print(f"错误: 目录 {RELEASES_DIR} 不存在")
            sys.exit(1)

        zip_files = [os.path.join(RELEASES_DIR, f) for f in os.listdir(RELEASES_DIR)
                     if f.endswith('.zip') and f.startswith('v')]

        if not zip_files:
            print(f"在 {RELEASES_DIR} 目录下未找到zip文件")
            sys.exit(1)

        files = sorted(zip_files)
    elif args.files:
        files = args.files
    else:
        parser.print_help()
        sys.exit(1)

    # 准备 body 内容
    body = None
    if args.body:
        body = args.body
    elif args.body_file:
        body = read_body_from_file(args.body_file)
    elif args.auto_notes:
        # 为每个文件生成对应的 release notes
        pass  # 将在循环中为每个文件单独生成

    target_commitish = args.target or subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()

    # 同版本的多个 BSP 必须发布到同一个 Release。
    releases = {}
    invalid_files = 0
    for file_path in files:
        if not os.path.exists(file_path):
            print(f"错误: 文件不存在: {file_path}")
            invalid_files += 1
            continue

        tag, _ = get_release_info(file_path)
        if not tag:
            print(f"错误: 文件名格式不正确: {file_path}")
            invalid_files += 1
            continue
        releases.setdefault(tag, []).append(file_path)

    success_count = 0
    for tag, release_files in sorted(releases.items()):
        try:
            release_body = body
            if args.auto_notes:
                release_body = generate_release_notes(tag, args.previous_tag)
                if release_body:
                    print(f"自动生成release notes:\n{release_body}\n")

            if publish_release(
                release_files,
                owner,
                repo,
                tag,
                token,
                target_commitish,
                draft=args.draft,
                prerelease=args.prerelease,
                overwrite=args.overwrite,
                body=release_body,
            ):
                success_count += 1
        except Exception as e:
            print(f"发布 {tag} 时出错: {e}")
            continue

    print(f"\n完成! 成功发布 {success_count}/{len(releases)} 个 Release")
    if invalid_files or success_count != len(releases):
        sys.exit(1)


if __name__ == "__main__":
    main()
