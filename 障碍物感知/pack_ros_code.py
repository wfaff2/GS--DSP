import os

# 配置：只抓取核心逻辑
ALLOWED_EXTENSIONS = {'.cpp', '.h', '.hpp', '.py', '.yaml', '.msg', '.srv'}
IGNORE_DIRS = {'build', 'devel', 'install', '.git', 'logs'}
MAX_SIZE_BYTES = 9.5 * 1024 * 1024  # 严格限制在 9.5MB 以内

def pack_with_splitting():
    cwd = os.getcwd()
    part_idx = 1
    current_size = 0
    
    # 初始打开第一个分卷
    def open_part(idx):
        return open(f"ros_logic_part{idx}.txt", 'w', encoding='utf-8')

    out_f = open_part(part_idx)
    print(f"开始生成分卷 1...")

    for root, dirs, files in os.walk(cwd):
        dirs[:] = [d for d in dirs if d not in IGNORE_DIRS]
        for file in files:
            if os.path.splitext(file)[1] in ALLOWED_EXTENSIONS:
                file_path = os.path.join(root, file)
                rel_path = os.path.relpath(file_path, cwd)
                
                try:
                    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                        content = f.read()
                        # 构造该文件的块内容
                        header = f"\n\n{'='*20}\nFile: {rel_path}\n{'='*20}\n"
                        file_block = header + content
                        block_bytes = file_block.encode('utf-8')

                        # 检查是否溢出：如果加上这一块超过了 9.5MB，就换新文件
                        if current_size + len(block_bytes) > MAX_SIZE_BYTES:
                            out_f.close()
                            part_idx += 1
                            out_f = open_part(part_idx)
                            current_size = 0
                            print(f"--- 达到限额，开启分卷 {part_idx} ---")

                        out_f.write(file_block)
                        current_size += len(block_bytes)
                except Exception as e:
                    print(f"跳过文件 {rel_path}: {e}")

    out_f.close()
    print(f"处理完成！共生成 {part_idx} 个分卷文件。")

if __name__ == "__main__":
    pack_with_splitting()
