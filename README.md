# ESP ROBOTICS

## Getting Started

**Requirements**
- Knowledge of C programming
- Knowledge about ESP IDF usage
- Knowledge of how git works?
- A PC with Linux / Windows installed
- A esp board to run the code
- [ESP IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)
 installed properly on system 

**Setup**
- Clone this repository
- Voila! you have the APIs ready to use
- NOTE: The API's visiblity is controlled via Kconfig so enable what's needed as required

## Code Formatting
This project uses `clang-format` for consistent code formatting. The configuration file `.clang-format` is provided in the root directory.

To format your code before committing:
```bash
# Format a single file
clang-format -i main/your_file.c

# Format all C/C++ files in main directory
find main -name "*.c" -o -name "*.h" | xargs clang-format -i
```

## Contributing
- Contributions are welcome! 
- while reporting a **BUG** provide appropriate logs along with it (USE THE ISSUES SECTION NOT MAIL)
- Adding of some new APIs you think contribute to world betterment? Make a pull request and let the world have it

## License
This project is licensed under the [MIT License](LICENSE).
