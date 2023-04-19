data = input('Enter data: ')
file_obj = open('log.txt', 'w+')
file_obj.write(data)
file_obj.close()

new_file = open('log.txt', 'r')
retrieved_data = new_file.read()
print(retrieved_data)