import re
from typing import List, Dict, Any
import tiktoken

class ContentChunker:
    def __init__(self, max_tokens: int = 512):
        """
        Initialize the content chunker with max tokens per chunk
        """
        self.max_tokens = max_tokens
        self.tokenizer = tiktoken.get_encoding("cl100k_base")  # Good for most text

    def count_tokens(self, text: str) -> int:
        """
        Count the number of tokens in a text
        """
        return len(self.tokenizer.encode(text))

    def chunk_text(self, text: str, source_path: str = "", chunk_id_prefix: str = "") -> List[Dict[str, Any]]:
        """
        Chunk text into segments of max_tokens or less
        """
        if not text.strip():
            return []

        # Split text into sentences to avoid cutting in the middle of sentences
        sentences = re.split(r'(?<=[.!?])\s+', text)

        chunks = []
        current_chunk = ""
        current_token_count = 0
        chunk_index = 0

        for sentence in sentences:
            sentence_token_count = self.count_tokens(sentence)

            # If a single sentence is too long, we need to break it down
            if sentence_token_count > self.max_tokens:
                # Split the long sentence into smaller parts
                sub_chunks = self._split_long_sentence(sentence)
                for sub_chunk in sub_chunks:
                    sub_chunk_token_count = self.count_tokens(sub_chunk)

                    if current_token_count + sub_chunk_token_count > self.max_tokens and current_chunk:
                        # Save current chunk and start new one
                        chunk_id = f"{chunk_id_prefix}_{chunk_index}" if chunk_id_prefix else str(chunk_index)
                        chunks.append({
                            "id": chunk_id,
                            "content": current_chunk.strip(),
                            "token_count": current_token_count,
                            "source_path": source_path,
                            "chunk_index": chunk_index
                        })

                        current_chunk = sub_chunk
                        current_token_count = sub_chunk_token_count
                        chunk_index += 1
                    else:
                        # Add to current chunk
                        if current_chunk:
                            current_chunk += " " + sub_chunk
                        else:
                            current_chunk = sub_chunk
                        current_token_count += sub_chunk_token_count
            else:
                # Check if adding this sentence would exceed the limit
                if current_token_count + sentence_token_count > self.max_tokens and current_chunk:
                    # Save current chunk and start new one
                    chunk_id = f"{chunk_id_prefix}_{chunk_index}" if chunk_id_prefix else str(chunk_index)
                    chunks.append({
                        "id": chunk_id,
                        "content": current_chunk.strip(),
                        "token_count": current_token_count,
                        "source_path": source_path,
                        "chunk_index": chunk_index
                    })

                    current_chunk = sentence
                    current_token_count = sentence_token_count
                    chunk_index += 1
                else:
                    # Add sentence to current chunk
                    if current_chunk:
                        current_chunk += " " + sentence
                    else:
                        current_chunk = sentence
                    current_token_count += sentence_token_count

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunk_id = f"{chunk_id_prefix}_{chunk_index}" if chunk_id_prefix else str(chunk_index)
            chunks.append({
                "id": chunk_id,
                "content": current_chunk.strip(),
                "token_count": current_token_count,
                "source_path": source_path,
                "chunk_index": chunk_index
            })

        return chunks

    def _split_long_sentence(self, sentence: str) -> List[str]:
        """
        Split a sentence that is too long into smaller parts
        """
        if self.count_tokens(sentence) <= self.max_tokens:
            return [sentence]

        # Try to split by commas first
        parts = sentence.split(', ')
        if all(self.count_tokens(part) <= self.max_tokens for part in parts):
            return [part.strip() + ', ' if i < len(parts) - 1 else part.strip()
                    for i, part in enumerate(parts)]

        # If comma splitting doesn't work, split by words
        words = sentence.split()
        chunks = []
        current_chunk = ""

        for word in words:
            test_chunk = current_chunk + " " + word if current_chunk else word
            if self.count_tokens(test_chunk) <= self.max_tokens:
                current_chunk = test_chunk
            else:
                if current_chunk:  # If there's something to save
                    chunks.append(current_chunk.strip())
                current_chunk = word

        if current_chunk:  # Add the last chunk
            chunks.append(current_chunk.strip())

        return chunks

    def chunk_markdown(self, markdown_content: str, source_path: str = "") -> List[Dict[str, Any]]:
        """
        Chunk markdown content preserving section structure where possible
        """
        # Split by markdown headers to keep sections together when possible
        header_pattern = r'^(#{1,6})\s+(.+)$'
        lines = markdown_content.split('\n')

        sections = []
        current_section = {'header': '', 'content': '', 'level': 0}

        for line in lines:
            header_match = re.match(header_pattern, line.strip())
            if header_match:
                # Save current section if it has content
                if current_section['content'].strip():
                    sections.append({
                        'header': current_section['header'],
                        'content': current_section['content'].strip(),
                        'level': current_section['level']
                    })

                # Start new section
                header_level = len(header_match.group(1))
                header_text = header_match.group(2)
                current_section = {
                    'header': header_text,
                    'content': f"{'#' * header_level} {header_text}\n\n",
                    'level': header_level
                }
            else:
                current_section['content'] += line + '\n'

        # Add the last section
        if current_section['content'].strip():
            sections.append({
                'header': current_section['header'],
                'content': current_section['content'].strip(),
                'level': current_section['level']
            })

        # Now chunk each section
        all_chunks = []
        for i, section in enumerate(sections):
            section_content = section['content']
            section_chunks = self.chunk_text(section_content, source_path, f"section_{i}")

            # Add section metadata to each chunk
            for chunk in section_chunks:
                chunk['section_header'] = section['header']
                chunk['section_level'] = section['level']
                all_chunks.append(chunk)

        return all_chunks