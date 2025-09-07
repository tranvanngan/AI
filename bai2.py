def solve_n_queens(n):
    """
    Tìm tất cả các giải pháp cho bài toán N-Queens bằng backtracking.
    """
    
    # Một mảng để lưu trữ vị trí cột của quân hậu trên mỗi hàng.
    # Ví dụ: board[0] = 2 nghĩa là quân hậu ở hàng 0, cột 2.
    board = [-1] * n  
    
    # Danh sách để lưu trữ tất cả các giải pháp tìm được.
    all_solutions = []

    def is_safe(row, col):
        """
        Kiểm tra xem vị trí (row, col) có an toàn để đặt quân hậu không.
        Ta chỉ cần kiểm tra các hàng phía trên, vì thuật toán đi từ trên xuống.
        """
        for prev_row in range(row):
            prev_col = board[prev_row]
            
            # Kiểm tra cùng cột hoặc cùng đường chéo
            if prev_col == col or abs(row - prev_row) == abs(col - prev_col):
                return False
        return True

    def backtrack(row):
        """
        Hàm đệ quy để tìm kiếm giải pháp.
        """
        # Điều kiện dừng: nếu đã đặt được N quân hậu (đã đi hết các hàng)
        if row == n:
            # Tạo một bản sao của bàn cờ và thêm vào danh sách giải pháp
            solution = []
            for col in board:
                row_str = '.' * n
                solution.append(row_str[:col] + 'Q' + row_str[col+1:])
            all_solutions.append(solution)
            return

        # Thử đặt quân hậu vào từng cột của hàng hiện tại
        for col in range(n):
            if is_safe(row, col):
                # Đặt quân hậu vào vị trí hợp lệ
                board[row] = col
                # Tiếp tục tìm kiếm giải pháp cho hàng tiếp theo
                backtrack(row + 1)
                # Quay lui: xóa quân hậu để thử vị trí khác
                board[row] = -1

    # Bắt đầu quá trình tìm kiếm từ hàng đầu tiên
    backtrack(0)
    return all_solutions

# --- Ví dụ sử dụng ---
n_value = 8
solutions = solve_n_queens(n_value)

print(f"Có tổng cộng {len(solutions)} giải pháp cho bài toán {n_value}-Queens.")
print("\nCác giải pháp:")
for i, solution in enumerate(solutions):
    print(f"\nGiải pháp {i+1}:")
    for row in solution:
        print(row)