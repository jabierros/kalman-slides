#!/bin/bash

# 1. Ask for a commit message if one wasn't provided
COMMIT_MESSAGE=$1
if [ -z "$COMMIT_MESSAGE" ]; then
    read -p "Enter your commit message: " COMMIT_MESSAGE
fi

echo "----------------------------------------"
echo "📂 Copying SVGs to web folder..."
# 2. Copy the files from the parent folder's Excalidraw directory
# The '../' tells the script to go up one level to look for the source files
cp -f ../Excalidraw/*.svg Excalidraw/

echo "📦 Staging changes..."
# 3. Add all changes in the repo (including this script itself!)
git add .

echo "💾 Committing..."
# 4. Commit with your message
git commit -m "$COMMIT_MESSAGE"

echo "🚀 Pushing to GitHub..."
# 5. Push to the live website
git push

echo "----------------------------------------"
echo "✅ Done! Give GitHub Pages ~60 seconds to build the new slides."
