#!/usr/bin/env python3
"""
Database migration script to update conversations table
- Makes user_id nullable
- Adds session_id column for anonymous users
"""
import asyncio
import sys
from pathlib import Path

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
load_dotenv()

from sqlalchemy import text
from db.postgres_client import get_engine

async def run_migration():
    """Run database migration"""
    print("Starting database migration...")

    engine = get_engine()

    async with engine.begin() as conn:
        # Step 1: Add session_id column if not exists
        print("Adding session_id column...")
        try:
            await conn.execute(text("""
                ALTER TABLE conversations
                ADD COLUMN IF NOT EXISTS session_id VARCHAR(100);
            """))
            print("✅ session_id column added")
        except Exception as e:
            print(f"⚠️ session_id column might already exist: {e}")

        # Step 2: Drop the existing foreign key constraint
        print("Dropping old foreign key constraint...")
        try:
            await conn.execute(text("""
                ALTER TABLE conversations
                DROP CONSTRAINT IF EXISTS conversations_user_id_fkey;
            """))
            print("✅ Old constraint dropped")
        except Exception as e:
            print(f"⚠️ Could not drop constraint: {e}")

        # Step 3: Make user_id nullable
        print("Making user_id nullable...")
        try:
            await conn.execute(text("""
                ALTER TABLE conversations
                ALTER COLUMN user_id DROP NOT NULL;
            """))
            print("✅ user_id is now nullable")
        except Exception as e:
            print(f"⚠️ user_id might already be nullable: {e}")

        # Step 4: Add new foreign key with ON DELETE SET NULL
        print("Adding new foreign key constraint...")
        try:
            await conn.execute(text("""
                ALTER TABLE conversations
                ADD CONSTRAINT conversations_user_id_fkey
                FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE SET NULL;
            """))
            print("✅ New constraint added")
        except Exception as e:
            print(f"⚠️ Constraint might already exist: {e}")

    print("\n" + "="*50)
    print("Migration completed!")
    print("="*50)

if __name__ == "__main__":
    asyncio.run(run_migration())
