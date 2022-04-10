import os

folder_path = "/qbrain/wwl-erasor/data_dir/pcds"
files = os.listdir(folder_path)
files.sort()
# print(files)

for file in files:
    oldname = os.path.join(folder_path, file)
    print(oldname)

    timestamp_old = file.split('.')[0]
    print(timestamp_old)

    # timestamp_old = '%f' %timestamp_old
    timestamp_new = '%06d' %int(timestamp_old)

    newname = os.path.join(folder_path, str(timestamp_new) + '.pcd')
    print(newname)
    os.rename(oldname, newname)
    print(oldname +'======>'+ newname)
